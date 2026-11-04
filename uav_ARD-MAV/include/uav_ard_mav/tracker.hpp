#pragma once

#include <memory>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <vector>

namespace ard_mav {

struct Settings {
  int imgsz = 640;
  float global_conf = 0.5F, local_conf = 0.1F;
  int crop_size = 300, roi_growth = 4, local_patience = 30;
  double process_noise = 0.01, measurement_noise = 1.0, initial_covariance = 1.0;
  int grid_columns = 30, grid_rows = 20, min_matches = 9;
  double ransac_threshold = 3.0, difference_threshold = 5.0;
  double light_coefficient = 1.0, background_coefficient = 0.1;
  int min_area = 30;
  double merge_distance = 15.0, angle_variance = 0.8, speed_variance = 0.8, min_speed = 1.0;
  int open_iterations = 1, close_iterations = 3;
  std::string provider = "cuda";
  int device_id = 0, threads = 4;
  void validate() const;
};

struct Detection {
  cv::Rect2f box;
  float score;
  cv::Point2f center() const { return {box.x + box.width / 2, box.y + box.height / 2}; }
};

// One float32 input and output. Owns ORT through a pimpl so consumers need only OpenCV.
class OnnxModel {
 public:
  OnnxModel(const std::string &path, const Settings &settings);
  ~OnnxModel();
  std::vector<float> run(const cv::Mat &blob, std::vector<int64_t> &shape);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

class AppearanceDetector {
 public:
  AppearanceDetector(const std::string &path, const Settings &settings);
  std::vector<Detection> detect_all(const cv::Mat &image, float confidence);
  std::optional<Detection> detect(const cv::Mat &image, float confidence,
                                  std::optional<cv::Point2f> reference = std::nullopt);

 private:
  OnnxModel model_;
  int imgsz_;
};

struct Homography {
  cv::Mat matrix;
  double background_motion = 0.0;
};
Homography estimate_homography(const cv::Mat &previous, const cv::Mat &current, const Settings &settings);
std::vector<cv::Rect> merge_regions(std::vector<cv::Rect> boxes, double distance);
cv::Rect search_region(cv::Point2f center, int lost, cv::Size size, const Settings &settings);

class RelativeVelocity {
 public:
  RelativeVelocity(cv::Point2f center, const Settings &settings);
  std::pair<cv::Point2f, cv::Point2f> predict(const cv::Mat &homography);
  void correct(cv::Point2f center, cv::Point2f camera_center);

 private:
  cv::Point2f center_;
  cv::KalmanFilter filter_;
};

struct MotionDiagnostics {
  bool called = false, homography_ok = false;
  int candidates = 0, motion_pass = 0, cnn_pass = 0;
};
class MotionDetector {
 public:
  MotionDetector(const std::string &path, const Settings &settings);
  std::optional<Detection> detect(const cv::Mat &previous, const cv::Mat &current,
                                  MotionDiagnostics &diagnostics,
                                  std::optional<cv::Point2f> reference = std::nullopt);

 private:
  OnnxModel classifier_;
  Settings settings_;
};

struct Result {
  std::optional<Detection> detection;
  std::string mode = "GLOBAL", next_mode = "GLOBAL", branch = "NONE";
  int lost = 0;
  std::optional<cv::Rect> roi;
  std::optional<cv::Point2f> predicted;
  bool homography_ok = false;
  MotionDiagnostics motion;
};

class Tracker {
 public:
  Tracker(const std::string &global, const std::string &local, const std::string &classifier,
          const Settings &settings);
  Result process(const cv::Mat &frame);
  void reset();

 private:
  Settings settings_;
  AppearanceDetector global_, local_;
  MotionDetector motion_;
  std::unique_ptr<RelativeVelocity> state_;
  cv::Mat previous_, previous_gray_;
  int lost_ = 0;
  // Alternate full-frame search models while no target is being tracked.
  bool search_local_ = false;
};
cv::Mat draw_result(const cv::Mat &frame, const Result &result);

}  // namespace ard_mav
