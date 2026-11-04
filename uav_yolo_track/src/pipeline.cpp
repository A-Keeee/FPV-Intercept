#include "uav_yolo_track/pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace yolo_track {
namespace {
Settings single_model_settings(Settings settings) {
  // The shared inference validator also checks the unused local threshold.
  settings.inference.local_conf = settings.inference.global_conf;
  return settings;
}
}  // namespace

Settings default_settings(const std::string &share) {
  Settings settings;
  settings.python = UAV_YOLO_PYTHON;
  settings.worker = share + "/scripts/tracker_worker.py";
  settings.tracker_config = share + "/config/botsort.yaml";
  return settings;
}

void Settings::validate() const {
  inference.validate();
  if (crop_size < 1 || roi_growth < 0 || local_patience < 1 || backend_timeout_ms < 1)
    throw std::invalid_argument("Invalid crop_size/roi_growth/local_patience/backend_timeout_ms");
}

cv::Rect search_region(const cv::Rect2f &box, int lost, cv::Size image, const Settings &settings) {
  const int side = int(std::min({double(settings.crop_size) + double(lost) * settings.roi_growth,
                                 double(image.width), double(image.height)}));
  const double x = double(box.x) + box.width / 2.0, y = double(box.y) + box.height / 2.0;
  return {int(std::clamp(std::nearbyint(x - side / 2.0), 0.0, double(image.width - side))),
          int(std::clamp(std::nearbyint(y - side / 2.0), 0.0, double(image.height - side))), side, side};
}

Pipeline::Pipeline(const std::string &global, const std::string &local, const Settings &settings)
    : settings_(settings),
      global_(global, settings.inference),
      local_(local.empty() ? nullptr
                           : std::make_unique<ard_mav::AppearanceDetector>(local, settings.inference)),
      tracker_(settings.python, settings.worker, settings.tracker_config, settings.backend_timeout_ms) {
  settings_.validate();
}

Pipeline::Pipeline(const std::string &model, const Settings &settings)
    : Pipeline(model, "", single_model_settings(settings)) {}

void Pipeline::reset() {
  target_.reset();
  lost_ = 0;
  previous_size_ = {};
  tracker_.reset();
}

Result Pipeline::process(const cv::Mat &frame) {
  if (frame.empty() || frame.type() != CV_8UC3) throw std::invalid_argument("Expected BGR8 frame");
  if (previous_size_.area() && frame.size() != previous_size_) reset();
  previous_size_ = frame.size();
  Result result;
  const bool single_model = !local_;
  const bool locked = bool(target_);
  const bool local = !single_model && locked;
  result.mode = single_model ? "SINGLE" : (local ? "LOCAL" : "GLOBAL");
  cv::Mat input = frame;
  if (local) {
    // Search around the last observed track, without an application-side velocity/Kalman predictor.
    result.roi = search_region(target_->box, lost_, frame.size(), settings_);
    input = frame(*result.roi);
  }
  auto detections =
      (local ? *local_ : global_)
          .detect_all(input, local ? settings_.inference.local_conf : settings_.inference.global_conf);
  for (auto &detection : detections) {
    if (result.roi) {
      detection.box.x += result.roi->x;
      detection.box.y += result.roi->y;
    }
  }
  result.detections = int(detections.size());
  // One tracker, one full-frame-coordinate update per processed frame, including misses.
  const auto tracks = tracker_.update(frame, detections);
  for (const auto &track : tracks) {
    if (locked && track.id != target_->id) continue;
    auto clipped = track;
    clipped.box &= cv::Rect2f(0, 0, float(frame.cols), float(frame.rows));
    if (clipped.box.area() > 0 && (!result.target || clipped.confidence > result.target->confidence))
      result.target = clipped;
  }
  if (result.target) {
    target_ = result.target;
    lost_ = 0;
  } else if (locked) {
    if (++lost_ >= settings_.local_patience) target_.reset();
  }
  result.lost = lost_;
  result.next_mode = single_model ? "SINGLE" : (target_ ? "LOCAL" : "GLOBAL");
  return result;
}

cv::Mat draw_result(const cv::Mat &frame, const Result &result) {
  auto canvas = frame.clone();
  if (result.roi) cv::rectangle(canvas, *result.roi, {255, 255, 255}, 1);
  if (result.target) {
    const auto &t = *result.target;
    cv::rectangle(canvas, t.box, {0, 220, 0}, 2);
    const auto label = cv::format("Drone ID:%d %.2f", t.id, t.confidence);
    const cv::Point origin{int(t.box.x), std::max(50, int(t.box.y) - 6)};
    cv::putText(canvas, label, origin, cv::FONT_HERSHEY_SIMPLEX, .6, {0, 0, 0}, 4, cv::LINE_AA);
    cv::putText(canvas, label, origin, cv::FONT_HERSHEY_SIMPLEX, .6, {0, 220, 0}, 2, cv::LINE_AA);
  }
  const auto label = result.mode + " | ID " + (result.target ? std::to_string(result.target->id) : "--") +
                     " | conf " + (result.target ? cv::format("%.2f", result.target->confidence) : "--") +
                     " | lost " + std::to_string(result.lost);
  cv::putText(canvas, label, {15, 60}, cv::FONT_HERSHEY_SIMPLEX, .7, {0, 0, 0}, 4, cv::LINE_AA);
  cv::putText(canvas, label, {15, 60}, cv::FONT_HERSHEY_SIMPLEX, .7, {0, 255, 255}, 2, cv::LINE_AA);
  return canvas;
}

}  // namespace yolo_track
