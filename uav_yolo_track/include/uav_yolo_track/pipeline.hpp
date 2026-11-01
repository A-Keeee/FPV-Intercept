#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <uav_ard_mav/tracker.hpp>
#include <vector>

namespace yolo_track {

struct Track {
  cv::Rect2f box;
  int id = -1;
  float confidence = 0;
};

// Only native Ultralytics tracker code maintains IDs and its internal motion model.
class NativeTracker {
 public:
  NativeTracker(std::string python, std::string script, std::string config, int timeout_ms = 5000);
  ~NativeTracker();
  NativeTracker(const NativeTracker &) = delete;
  NativeTracker &operator=(const NativeTracker &) = delete;
  std::vector<Track> update(const cv::Mat &frame, const std::vector<ard_mav::Detection> &detections);
  void reset();

 private:
  void start();
  void stop() noexcept;
  void transfer(void *data, size_t bytes, bool writing, std::chrono::steady_clock::time_point deadline);
  std::vector<Track> exchange(const cv::Mat &frame, const std::vector<ard_mav::Detection> &detections,
                              bool reset);
  std::string python_, script_, config_;
  int timeout_ms_, socket_ = -1, pid_ = -1;
};

struct Settings {
  Settings() { inference.global_conf = 0.3F; }
  ard_mav::Settings inference;
  int crop_size = 300, roi_growth = 4, local_patience = 30, backend_timeout_ms = 5000;
  std::string python, worker, tracker_config;
  void validate() const;
};
Settings default_settings(const std::string &share);

struct Result {
  std::string mode = "GLOBAL", next_mode = "GLOBAL";
  std::optional<Track> target;
  std::optional<cv::Rect> roi;
  int lost = 0, detections = 0;
};

cv::Rect search_region(const cv::Rect2f &last_box, int lost, cv::Size image, const Settings &settings);
class Pipeline {
 public:
  Pipeline(const std::string &global_model, const std::string &local_model, const Settings &settings);
  Pipeline(const std::string &model, const Settings &settings);
  Result process(const cv::Mat &frame);
  void reset();

 private:
  Settings settings_;
  ard_mav::AppearanceDetector global_;
  std::unique_ptr<ard_mav::AppearanceDetector> local_;
  NativeTracker tracker_;
  std::optional<Track> target_;
  cv::Size previous_size_;
  int lost_ = 0;
};

cv::Mat draw_result(const cv::Mat &frame, const Result &result);

}  // namespace yolo_track
