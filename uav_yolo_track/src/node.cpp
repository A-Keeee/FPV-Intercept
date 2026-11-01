#include <cv_bridge/cv_bridge.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <thread>
#include <uav_common_msg/msg/rect_msg.hpp>

#include "uav_yolo_track/pipeline.hpp"

using Clock = std::chrono::steady_clock;
using RectMsg = uav_common_msg::msg::RectMsg;

#ifdef UAV_YOLO_SINGLE_MODEL
constexpr bool kSingleModel = true;
#else
constexpr bool kSingleModel = false;
#endif

class YoloTrackNode : public rclcpp::Node {
 public:
  YoloTrackNode() : Node(kSingleModel ? "uav_yolo_single_track" : "uav_yolo_track") {
    const auto share = ament_index_cpp::get_package_share_directory("uav_yolo_track");
    const auto model_dir =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("uav_ard_mav")) / "models";
    auto settings = yolo_track::default_settings(share);
    settings.inference.imgsz = declare_parameter("imgsz", 640);
    if (kSingleModel) {
      settings.inference.global_conf = float(declare_parameter("conf", 0.3));
      settings.inference.local_conf = settings.inference.global_conf;
      settings.local_patience = declare_parameter("target_patience", 30);
    } else {
      settings.inference.global_conf = float(declare_parameter("global_conf", 0.3));
      settings.inference.local_conf = float(declare_parameter("local_conf", 0.1));
      settings.crop_size = declare_parameter("crop_size", 300);
      settings.roi_growth = declare_parameter("roi_growth", 4);
      settings.local_patience = declare_parameter("local_patience", 30);
    }
    settings.inference.provider = declare_parameter<std::string>("provider", "cpu");
    settings.inference.device_id = declare_parameter("device_id", 0);
    settings.inference.threads = declare_parameter("threads", 4);
    settings.backend_timeout_ms = declare_parameter("backend_timeout_ms", 5000);
    settings.python = declare_parameter<std::string>("python_executable", settings.python);
    auto config =
        std::filesystem::path(declare_parameter<std::string>("tracker_config", "config/botsort.yaml"));
    if (config.is_relative()) config = std::filesystem::path(share) / config;
    settings.tracker_config = config.string();
    settings.validate();
    auto model = [&](const std::string &key, const std::string &filename) {
      auto path = std::filesystem::path(declare_parameter<std::string>(key, filename));
      return (path.is_relative() ? model_dir / path : path).string();
    };
    timeout_ = declare_parameter("image_timeout", 0.5);
    const double rate = declare_parameter("publish_rate", 60.0);
    show_ = declare_parameter("show", true);
    if (!std::isfinite(timeout_) || timeout_ <= 0 || !std::isfinite(rate) || rate <= 0 || rate > 1000)
      throw std::invalid_argument("Require image_timeout > 0 and 0 < publish_rate <= 1000");
    if (kSingleModel) {
      const auto path = model("model", "global_best.onnx");
      pipeline_ = std::make_unique<yolo_track::Pipeline>(path, settings);
      RCLCPP_INFO(get_logger(), "YOLO single-model full-frame + native tracker ready; model: %s",
                  path.c_str());
    } else {
      const auto global = model("global_model", "global_best.onnx");
      const auto local = model("local_model", "local_multiscale_ft_20261030.onnx");
      pipeline_ = std::make_unique<yolo_track::Pipeline>(global, local, settings);
      RCLCPP_INFO(get_logger(), "YOLO global/local + native tracker ready; global: %s; local: %s",
                  global.c_str(), local.c_str());
    }
    results_ = create_publisher<RectMsg>(
        declare_parameter<std::string>("result_topic", "/camera_detect_result"), 10);
    ids_ = create_publisher<std_msgs::msg::Int32>("~/target_id", 10);
    confidences_ = create_publisher<std_msgs::msg::Float32>("~/confidence", 10);
    debug_ = create_publisher<sensor_msgs::msg::Image>("~/debug_image", rclcpp::SensorDataQoS());
    const auto image_topic = declare_parameter<std::string>("image_topic", "/camera/image");
    images_ =
        create_subscription<sensor_msgs::msg::Image>(image_topic, rclcpp::SensorDataQoS().keep_last(1),
                                                     [this](sensor_msgs::msg::Image::ConstSharedPtr image) {
                                                       {
                                                         std::lock_guard<std::mutex> lock(mutex_);
                                                         pending_ = std::move(image);
                                                         pending_time_ = Clock::now();
                                                       }
                                                       condition_.notify_one();
                                                     });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() { publish(); });
    RCLCPP_INFO(get_logger(), "Tracker: %s; no application Kalman predictor", config.c_str());
    worker_ = std::thread([this]() { work(); });
  }

  ~YoloTrackNode() override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    condition_.notify_one();
    if (worker_.joinable()) worker_.join();
  }

 private:
  static RectMsg invalid(const std_msgs::msg::Header &header) {
    RectMsg result;
    result.header = header;
    result.x = result.y = result.width = result.height = -1;
    result.depth = 0;
    return result;
  }

  void publish() {
    RectMsg rectangle;
    std_msgs::msg::Int32 id;
    std_msgs::msg::Float32 confidence;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      rectangle = latest_;
      if (!ready_ || std::chrono::duration<double>(Clock::now() - latest_time_).count() > timeout_) {
        rectangle = invalid(rectangle.header);
        id.data = -1;
      } else {
        id.data = latest_id_;
        confidence.data = latest_confidence_;
      }
      if (!ready_) rectangle.header.stamp = now();
    }
    results_->publish(rectangle);
    ids_->publish(id);
    confidences_->publish(confidence);
  }

  void work() {
    std::optional<rclcpp::Time> previous_stamp;
    Clock::time_point previous_time;
    while (true) {
      sensor_msgs::msg::Image::ConstSharedPtr image;
      Clock::time_point received;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this]() { return stop_ || bool(pending_); });
        if (stop_) break;
        image = std::move(pending_);
        received = pending_time_;
      }
      auto rectangle = invalid(image->header);
      int id = -1;
      float confidence = 0;
      try {
        const rclcpp::Time stamp(image->header.stamp);
        if (previous_stamp && (stamp < *previous_stamp || (stamp - *previous_stamp).seconds() > timeout_ ||
                               std::chrono::duration<double>(received - previous_time).count() > timeout_))
          pipeline_->reset();
        previous_stamp = stamp;
        previous_time = received;
        const auto frame = cv_bridge::toCvShare(image, "bgr8");
        const auto result = pipeline_->process(frame->image);
        if (result.target) {
          const auto &target = *result.target;
          rectangle.x = int(std::floor(target.box.x));
          rectangle.y = int(std::floor(target.box.y));
          rectangle.width = int(std::ceil(target.box.x + target.box.width)) - rectangle.x;
          rectangle.height = int(std::ceil(target.box.y + target.box.height)) - rectangle.y;
          id = target.id;
          confidence = target.confidence;
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "%s -> %s | ID %d | conf %.2f | lost %d",
                             result.mode.c_str(), result.next_mode.c_str(), id, confidence, result.lost);
        if (show_ || debug_->get_subscription_count()) {
          const auto canvas = yolo_track::draw_result(frame->image, result);
          if (debug_->get_subscription_count())
            debug_->publish(*cv_bridge::CvImage(image->header, "bgr8", canvas).toImageMsg());
          if (show_) {
            cv::imshow(kSingleModel ? "YOLO Single Model Track" : "YOLO Global-Local Track", canvas);
            cv::waitKey(1);
          }
        }
      } catch (const std::exception &error) {
        rectangle = invalid(image->header);
        id = -1;
        confidence = 0;
        try {
          pipeline_->reset();
        } catch (const std::exception &) {
        }
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Frame rejected: %s", error.what());
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = rectangle;
        latest_time_ = received;
        latest_id_ = id;
        latest_confidence_ = confidence;
        ready_ = true;
      }
    }
    if (show_) cv::destroyAllWindows();
  }

  std::unique_ptr<yolo_track::Pipeline> pipeline_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr images_;
  rclcpp::Publisher<RectMsg>::SharedPtr results_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr ids_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr confidences_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread worker_;
  std::mutex mutex_;
  std::condition_variable condition_;
  sensor_msgs::msg::Image::ConstSharedPtr pending_;
  Clock::time_point pending_time_, latest_time_;
  RectMsg latest_;
  int latest_id_ = -1;
  float latest_confidence_ = 0;
  bool stop_ = false, ready_ = false, show_ = true;
  double timeout_ = 0.5;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    rclcpp::spin(std::make_shared<YoloTrackNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger(kSingleModel ? "uav_yolo_single_track" : "uav_yolo_track"), "%s",
                 error.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
