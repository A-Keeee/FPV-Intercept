#include "uav_ard_mav/tracker.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ard_mav {

cv::Rect search_region(cv::Point2f center, int lost, cv::Size size, const Settings &settings) {
  const int side = int(std::min({double(settings.crop_size) + double(lost) * settings.roi_growth,
                                 double(size.width), double(size.height)}));
  // nearbyint matches Python round's ties-to-even convention.
  const int x =
      int(std::clamp(std::nearbyint(double(center.x) - side / 2.0), 0.0, double(size.width - side)));
  const int y =
      int(std::clamp(std::nearbyint(double(center.y) - side / 2.0), 0.0, double(size.height - side)));
  return {x, y, side, side};
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
                 const Settings &settings)
    : settings_(settings),
      global_(global, settings),
      local_(local, settings),
      motion_(classifier, settings) {}

void Tracker::reset() {
  state_.reset();
  previous_.release();
  previous_gray_.release();
  lost_ = 0;
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
    result.mode = "LOCAL";
    const auto homography = estimate_homography(previous_gray_, gray, settings_);
    result.homography_ok = !homography.matrix.empty();
    const auto prediction = state_->predict(homography.matrix);
    result.predicted = prediction.first;
    camera_center = prediction.second;
    result.roi = search_region(*result.predicted, lost_, frame.size(), settings_);
    image = frame(*result.roi);
    previous_image = previous_(*result.roi);
    reference = *result.predicted - cv::Point2f(float(result.roi->x), float(result.roi->y));
  }
  const bool local = bool(state_);
  result.branch = local ? "LAD" : "GAD";
  result.detection = (local ? local_ : global_)
                         .detect(image, local ? settings_.local_conf : settings_.global_conf, reference);
  if (!result.detection && !previous_image.empty()) {
    result.branch = local ? "LMD" : "GMD";
    result.detection = motion_.detect(previous_image, image, result.motion, reference);
  }
  if (result.detection) {
    if (result.roi) {
      result.detection->box.x += result.roi->x;
      result.detection->box.y += result.roi->y;
    }
    const auto center = result.detection->center();
    if (!state_)
      state_ = std::make_unique<RelativeVelocity>(center, settings_);
    else
      state_->correct(center, camera_center);
    lost_ = 0;
  } else {
    result.branch = "NONE";
    if (local && ++lost_ >= settings_.local_patience) state_.reset();
  }
  result.lost = lost_;
  result.next_mode = state_ ? "LOCAL" : "GLOBAL";
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
              {15, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 255}, 2);
  return canvas;
}

}  // namespace ard_mav
