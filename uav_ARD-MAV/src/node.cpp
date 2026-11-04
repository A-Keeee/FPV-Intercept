#include <cv_bridge/cv_bridge.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>
#include <type_traits>
#include <uav_common_msg/msg/rect_msg.hpp>

#include "uav_ard_mav/tracker.hpp"

using SteadyClock = std::chrono::steady_clock;

class ArdMavNode : public rclcpp::Node {
 public:
  ArdMavNode() : Node("uav_ard_mav") {
    ard_mav::Settings settings;
    auto parameter = [this](auto &value, const std::string &name) {
      using T = std::decay_t<decltype(value)>;
      if constexpr (std::is_same_v<T, float>)
        value = float(declare_parameter<double>(name, value));
      else
        value = declare_parameter<T>(name, value);
    };
    parameter(settings.imgsz, "imgsz");
    parameter(settings.global_conf, "global_conf");
    parameter(settings.local_conf, "local_conf");
    parameter(settings.crop_size, "crop_size");
    parameter(settings.roi_growth, "roi_growth");
    parameter(settings.local_patience, "local_patience");
    parameter(settings.process_noise, "kalman.process_noise");
    parameter(settings.measurement_noise, "kalman.measurement_noise");
    parameter(settings.initial_covariance, "kalman.initial_covariance");
    parameter(settings.grid_columns, "motion.grid_columns");
    parameter(settings.grid_rows, "motion.grid_rows");
    parameter(settings.min_matches, "motion.min_matches");
    parameter(settings.ransac_threshold, "motion.ransac_threshold");
    parameter(settings.difference_threshold, "motion.difference_threshold");
    parameter(settings.light_coefficient, "motion.light_coefficient");
    parameter(settings.background_coefficient, "motion.background_coefficient");
    parameter(settings.min_area, "motion.min_area");
    parameter(settings.merge_distance, "motion.merge_distance");
    parameter(settings.angle_variance, "motion.angle_variance");
    parameter(settings.speed_variance, "motion.speed_variance");
    parameter(settings.min_speed, "motion.min_speed");
    parameter(settings.open_iterations, "motion.open_iterations");
    parameter(settings.close_iterations, "motion.close_iterations");
    parameter(settings.provider, "provider");
    parameter(settings.device_id, "device_id");
    parameter(settings.threads, "threads");
    settings.validate();
    const auto share = std::filesystem::path(ament_index_cpp::get_package_share_directory("uav_ard_mav"));
    auto model_path = [&](const std::string &key, const std::string &filename) {
      auto path = std::filesystem::path(declare_parameter<std::string>(key, "models/" + filename));
      if (path.is_relative()) path = share / path;
      return path.string();
    };
    const auto global = model_path("global_model", "global_best.onnx");
    const auto local = model_path("local_model", "local_multiscale_ft_20261030.onnx");
    const auto classifier = model_path("classifier_model", "Net_best.onnx");
    timeout_ = declare_parameter<double>("image_timeout", 0.5);
    const double rate = declare_parameter<double>("publish_rate", 60.0);
    show_ = declare_parameter<bool>("show", true);
    show_width_ = declare_parameter<int>("show_width", 960);
    show_height_ = declare_parameter<int>("show_height", 540);
    if (!std::isfinite(timeout_) || timeout_ <= 0 || !std::isfinite(rate) || rate <= 0 || rate > 1000)
      throw std::invalid_argument("image_timeout must be positive; publish_rate must be in (0,1000]");
    if (show_width_ < 1 || show_height_ < 1)
      throw std::invalid_argument("Require show_width > 0 and show_height > 0");
    tracker_ = std::make_unique<ard_mav::Tracker>(global, local, classifier, settings);
    const auto image_topic = declare_parameter<std::string>("image_topic", "/camera/image");
    publisher_ = create_publisher<uav_common_msg::msg::RectMsg>(
        declare_parameter<std::string>("result_topic", "/camera_detect_result"), 10);
    debug_ = create_publisher<sensor_msgs::msg::Image>(
        declare_parameter<std::string>("debug_topic", "/uav_ard_mav/debug_image"), rclcpp::SensorDataQoS());
    // The worker consumes the newest frame only; inference never blocks the result timer.
    subscription_ =
        create_subscription<sensor_msgs::msg::Image>(image_topic, rclcpp::SensorDataQoS().keep_last(1),
                                                     [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
                                                       {
                                                         std::lock_guard<std::mutex> lock(mutex_);
                                                         pending_ = std::move(message);
                                                         pending_time_ = SteadyClock::now();
                                                       }
                                                       condition_.notify_one();
                                                     });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() { publish(); });
    RCLCPP_INFO(get_logger(), "ARD-MAV ready (%s): %s -> %s", settings.provider.c_str(), image_topic.c_str(),
                publisher_->get_topic_name());
    worker_ = std::thread([this]() { work(); });
  }

  ~ArdMavNode() override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    condition_.notify_one();
    if (worker_.joinable()) worker_.join();
  }

 private:
  static uav_common_msg::msg::RectMsg invalid(const std_msgs::msg::Header &header) {
    uav_common_msg::msg::RectMsg message;
    message.header = header;
    message.x = message.y = message.width = message.height = -1;
    message.depth = 0;
    return message;
  }

  void publish() {
    uav_common_msg::msg::RectMsg message;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      message = result_;
      if (!has_result_ || std::chrono::duration<double>(SteadyClock::now() - result_time_).count() > timeout_)
        message = invalid(message.header);
      if (!has_result_) message.header.stamp = now();
    }
    publisher_->publish(message);
  }

  void work() {
    SteadyClock::time_point previous_time{};
    std::optional<rclcpp::Time> previous_stamp;
    bool window_initialized = false;
    std::optional<SteadyClock::time_point> fps_window_start;
    int fps_window_frames = 0;
    double fps = 0.0;
    while (true) {
      sensor_msgs::msg::Image::ConstSharedPtr image;
      SteadyClock::time_point received;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this]() { return stop_ || bool(pending_); });
        if (stop_) break;
        image = std::move(pending_);
        received = pending_time_;
      }
      auto message = invalid(image->header);
      try {
        const rclcpp::Time stamp(image->header.stamp);
        if (previous_stamp && (stamp < *previous_stamp || (stamp - *previous_stamp).seconds() > timeout_ ||
                               std::chrono::duration<double>(received - previous_time).count() > timeout_))
          tracker_->reset();
        previous_stamp = stamp;
        previous_time = received;
        const auto frame = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGR8);
        const auto result = tracker_->process(frame->image);
        const auto completed = SteadyClock::now();
        if (!fps_window_start) {
          fps_window_start = completed;
        } else {
          ++fps_window_frames;
          const double elapsed = std::chrono::duration<double>(completed - *fps_window_start).count();
          if (elapsed >= 0.5) {
            fps = fps_window_frames / elapsed;
            fps_window_frames = 0;
            fps_window_start = completed;
          }
        }
        if (result.detection) {
          const auto &box = result.detection->box;
          // Round outwards so integer publication preserves every observed box pixel.
          message.x = int(std::floor(box.x));
          message.y = int(std::floor(box.y));
          message.width = int(std::ceil(box.x + box.width)) - message.x;
          message.height = int(std::ceil(box.y + box.height)) - message.y;
        }
        if (result.mode == "LOCAL" && !result.homography_ok)
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                               "ROI homography failed; velocity prediction only");
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "%s/%s -> %s, lost=%d, motion=%d/%d/%d",
                             result.mode.c_str(), result.branch.c_str(), result.next_mode.c_str(),
                             result.lost, result.motion.candidates, result.motion.motion_pass,
                             result.motion.cnn_pass);
        if (show_ || debug_->get_subscription_count() > 0) {
          auto canvas = ard_mav::draw_result(frame->image, result);
          const auto fps_label = fps > 0 ? cv::format("FPS: %.1f", fps) : std::string("FPS: --");
          cv::putText(canvas, fps_label, {15, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 0, 0}, 4,
                      cv::LINE_AA);
          cv::putText(canvas, fps_label, {15, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {255, 255, 0}, 2,
                      cv::LINE_AA);
          if (debug_->get_subscription_count() > 0)
            debug_->publish(*cv_bridge::CvImage(image->header, "bgr8", canvas).toImageMsg());
          if (show_) {
            if (!window_initialized) {
              cv::namedWindow("ARD-MAV", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
              cv::resizeWindow("ARD-MAV", show_width_, show_height_);
              window_initialized = true;
            }
            cv::imshow("ARD-MAV", canvas);
            cv::waitKey(1);
          }
        }
      } catch (const std::exception &error) {
        tracker_->reset();
        message = invalid(image->header);
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Frame rejected: %s", error.what());
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        result_ = message;
        result_time_ = received;
        has_result_ = true;
      }
    }
    if (show_) cv::destroyAllWindows();
  }

  std::unique_ptr<ard_mav::Tracker> tracker_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<uav_common_msg::msg::RectMsg>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::thread worker_;
  sensor_msgs::msg::Image::ConstSharedPtr pending_;
  SteadyClock::time_point pending_time_, result_time_;
  uav_common_msg::msg::RectMsg result_;
  int show_width_ = 960, show_height_ = 540;
  bool stop_ = false, has_result_ = false, show_ = true;
  double timeout_ = 0.5;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    rclcpp::spin(std::make_shared<ArdMavNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("uav_ard_mav"), "%s", error.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
