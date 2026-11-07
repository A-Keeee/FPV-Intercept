#include "uav_ard_mav/tracker.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ard_mav {

cv::Rect search_region(cv::Point2f center, int lost, cv::Size size, const Settings &settings,
                       cv::Size2f target_size) {
  const double growth = double(lost) * settings.roi_growth;
  // Clamp each axis independently: a wide target can exceed the image's short
  // side. Keep target padding as well as the original missed-frame expansion.
  const int width = int(std::min(double(size.width),
      std::max(double(settings.crop_size), std::ceil(target_size.width * settings.roi_padding)) + growth));
  const int height = int(std::min(double(size.height),
      std::max(double(settings.crop_size), std::ceil(target_size.height * settings.roi_padding)) + growth));
  // nearbyint matches Python round's ties-to-even convention.
  const int x =
      int(std::clamp(std::nearbyint(double(center.x) - width / 2.0), 0.0, double(size.width - width)));
  const int y =
      int(std::clamp(std::nearbyint(double(center.y) - height / 2.0), 0.0, double(size.height - height)));
  return {x, y, width, height};
}

RelativeVelocity::RelativeVelocity(cv::Point2f center, const Settings &settings)
    : center_(center), filter_(4, 2) {
  filter_.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
  filter_.measurementMatrix = cv::Mat::eye(2, 4, CV_32F);
  cv::setIdentity(filter_.processNoiseCov, cv::Scalar(settings.process_noise));
  cv::setIdentity(filter_.measurementNoiseCov, cv::Scalar(settings.measurement_noise));
  cv::setIdentity(filter_.errorCovPost, cv::Scalar(settings.initial_covariance));
  filter_.statePost.setTo(0);
}

std::pair<cv::Point2f, cv::Point2f> RelativeVelocity::predict(const cv::Mat &homography) {
  auto camera = center_;
  if (!homography.empty()) {
    std::vector<cv::Point2f> points{center_}, transformed;
    cv::perspectiveTransform(points, transformed, homography);
    camera = transformed[0];
  }
  const cv::Point2f velocity{filter_.statePost.at<float>(0), filter_.statePost.at<float>(1)};
  filter_.predict();
  center_ = camera + velocity;
  if (!std::isfinite(center_.x) || !std::isfinite(center_.y))
    throw std::runtime_error("Nonfinite camera/velocity prediction");
  return {center_, camera};
}

void RelativeVelocity::correct(cv::Point2f center, cv::Point2f camera_center) {
  filter_.correct((cv::Mat_<float>(2, 1) << center.x - camera_center.x, center.y - camera_center.y));
  center_ = center;
}

Tracker::Tracker(const std::string &global, const std::string &local, const std::string &classifier,
                 const Settings &settings, const std::string &local_full_path)
    : settings_(settings),
      global_(global, settings),
      local_(local, settings),
      motion_(classifier, settings) {
  const auto &path = local_full_path.empty() ?
      (settings.local_full_model == "global" ? global : local) : local_full_path;
  if (path == global) {
    full_uses_global_ = true;
  } else if (path != local && settings.enable_local_full) {
    separate_full_model_ = std::make_unique<AppearanceDetector>(path, settings);
  }
}

void Tracker::reset() {
  state_.reset();
  roi_target_size_ = global_target_size_ = {};
  previous_.release();
  previous_gray_.release();
  lost_ = 0;
  search_frames_ = 0;
  full_frame_tracking_ = false;
}

Result Tracker::process(const cv::Mat &frame) {
  if (frame.empty() || frame.type() != CV_8UC3) throw std::invalid_argument("Expected a nonempty BGR8 frame");
  if (!previous_.empty() && frame.size() != previous_.size()) reset();
  Result result;
  cv::Mat gray;
  cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  cv::Mat image = frame, previous_image = previous_;
  std::optional<cv::Point2f> reference;
  cv::Point2f camera_center;
  if (state_) {
    result.mode = full_frame_tracking_ ? "LOCAL_FULL" : "LOCAL";
    const auto homography = estimate_homography(previous_gray_, gray, settings_);
    result.homography_ok = !homography.matrix.empty();
    const auto prediction = state_->predict(homography.matrix);
    result.predicted = prediction.first;
    camera_center = prediction.second;
    reference = result.predicted;
    if (!full_frame_tracking_) {
      const cv::Size2f minimum_size{std::max(roi_target_size_.width, global_target_size_.width),
                                    std::max(roi_target_size_.height, global_target_size_.height)};
      result.roi = search_region(*result.predicted, lost_, frame.size(), settings_, minimum_size);
      image = frame(*result.roi);
      previous_image = previous_(*result.roi);
      reference = *result.predicted - cv::Point2f(float(result.roi->x), float(result.roi->y));
    }
  }
  const bool local = bool(state_);
  const bool use_local_model = local || (search_frames_ + 1) % settings_.global_search_interval != 0;
  if (!local && use_local_model) result.mode = "LOCAL_MULTI";
  const bool full_frame = result.mode == "LOCAL_FULL";
  const bool use_global_threshold = full_frame ? settings_.local_full_model == "global" : !use_local_model;
  auto &full_detector = separate_full_model_ ? *separate_full_model_ :
                        (full_uses_global_ ? global_ : local_);
  auto &detector = full_frame ? full_detector : (use_local_model ? local_ : global_);
  result.branch = use_global_threshold ? "GAD" : "LAD";
  result.detection = detector.detect(image, use_global_threshold ? settings_.global_conf : settings_.local_conf,
                                     reference);
  if (settings_.motion_fallback && !result.detection && !previous_image.empty()) {
    result.branch = result.roi ? "LMD" : "GMD";
    result.detection = motion_.detect(previous_image, image, result.motion, reference);
  }
  if (result.detection) {
    if (result.roi) {
      result.detection->box.x += result.roi->x;
      result.detection->box.y += result.roi->y;
    }
    const auto &box = result.detection->box;
    // ROI detections may be clipped, so never shrink the remembered dimensions
    // from a cropped observation. Full-frame observations can measure shrinkage,
    // but a global detector's size remains a floor until this track ends.
    if (result.roi) {
      roi_target_size_.width = std::max(roi_target_size_.width, box.width);
      roi_target_size_.height = std::max(roi_target_size_.height, box.height);
    } else {
      roi_target_size_ = box.size();
    }
    if (result.branch == "GAD") {
      global_target_size_.width = std::max(global_target_size_.width, box.width);
      global_target_size_.height = std::max(global_target_size_.height, box.height);
    }
    // Ratios always use original-frame coordinates and the nominal crop size.
    result.target_fov_ratio = std::max(double(box.width) / frame.cols, double(box.height) / frame.rows);
    const double nominal_side = std::min({settings_.crop_size, frame.cols, frame.rows});
    result.target_roi_fill = std::max(box.width, box.height) / nominal_side;
    if (!settings_.enable_local_full) {
      full_frame_tracking_ = false;
    } else if (result.target_fov_ratio >= settings_.full_frame_enter_ratio ||
               result.target_roi_fill >= settings_.roi_fill_enter_ratio) {
      full_frame_tracking_ = true;
    } else if (result.target_fov_ratio <= settings_.full_frame_exit_ratio &&
               result.target_roi_fill <= settings_.roi_fill_exit_ratio) {
      full_frame_tracking_ = false;
    }
    const auto center = result.detection->center();
    if (!state_)
      state_ = std::make_unique<RelativeVelocity>(center, settings_);
    else
      state_->correct(center, camera_center);
    lost_ = 0;
    search_frames_ = 0;
  } else {
    result.branch = "NONE";
    if (local) {
      if (++lost_ >= settings_.local_patience) {
        state_.reset();
        roi_target_size_ = global_target_size_ = {};
        search_frames_ = 0;
        full_frame_tracking_ = false;
      }
    } else {
      search_frames_ = (search_frames_ + 1) % settings_.global_search_interval;
    }
  }
  result.lost = lost_;
  result.next_mode = state_ ? (full_frame_tracking_ ? "LOCAL_FULL" : "LOCAL") :
      ((search_frames_ + 1) % settings_.global_search_interval != 0 ? "LOCAL_MULTI" : "GLOBAL");
  previous_ = frame.clone();
  previous_gray_ = gray;
  return result;
}

cv::Mat draw_result(const cv::Mat &frame, const Result &result) {
  auto canvas = frame.clone();
  if (result.roi) cv::rectangle(canvas, *result.roi, {255, 255, 255}, 1);
  if (result.detection) {
    cv::rectangle(canvas, result.detection->box, {0, 220, 0}, 2);
    const auto label = cv::format("Drone %.2f", result.detection->score);
    int baseline = 0;
    const auto size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);
    const int x = std::clamp(int(result.detection->box.x), 0, std::max(0, frame.cols - size.width - 4));
    const int y = std::max(size.height + 4, int(result.detection->box.y) - 6);
    cv::rectangle(canvas, {x, y - size.height - 4, size.width + 4, size.height + baseline + 8}, {0, 0, 0},
                  cv::FILLED);
    cv::putText(canvas, label, {x + 2, y}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 220, 0}, 2, cv::LINE_AA);
  } else if (result.predicted) {
    const cv::Point point{int(std::clamp(result.predicted->x, 0.F, float(frame.cols - 1))),
                          int(std::clamp(result.predicted->y, 0.F, float(frame.rows - 1)))};
    cv::circle(canvas, point, 5, {0, 160, 255}, 1);
  }
  const auto confidence = result.detection ? cv::format("%.2f", result.detection->score) : std::string("--");
  cv::putText(canvas,
              result.mode + " | " + result.branch + " | conf " + confidence + " | lost " +
                  std::to_string(result.lost),
              {15, 60}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 255}, 2);
  if (result.detection) {
    cv::putText(canvas,
                cv::format("FOV %.1f%% | ROI fill %.1f%% | next %s", result.target_fov_ratio * 100,
                           result.target_roi_fill * 100, result.next_mode.c_str()),
                {15, 90}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 255}, 2);
  }
  if (result.mode == "LOCAL_FULL") {
    const std::string label = "LOCAL_FULL";
    constexpr int margin = 10, padding_x = 8, padding_y = 4;
    int baseline = 0;
    const auto text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.75, 2, &baseline);
    const cv::Size badge_size{text_size.width + 2 * padding_x, text_size.height + baseline + 2 * padding_y};
    const cv::Point origin{std::max(0, canvas.cols - margin - badge_size.width),
                            std::min(margin, std::max(0, canvas.rows - badge_size.height))};
    const cv::Rect badge{origin, badge_size};
    cv::rectangle(canvas, badge, {20, 20, 20}, cv::FILLED);
    cv::rectangle(canvas, badge, {0, 165, 255}, 2);
    cv::putText(canvas, label, origin + cv::Point(padding_x, padding_y + text_size.height),
                cv::FONT_HERSHEY_SIMPLEX, 0.75, {0, 220, 255}, 2, cv::LINE_AA);
  }
  return canvas;
}

}  // namespace ard_mav
