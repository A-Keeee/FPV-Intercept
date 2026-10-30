#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "uav_ard_mav/tracker.hpp"

namespace ard_mav {
namespace {
bool finite(cv::Point2f point) { return std::isfinite(point.x) && std::isfinite(point.y); }
}  // namespace

Homography estimate_homography(const cv::Mat &previous, const cv::Mat &current, const Settings &settings) {
  std::vector<cv::Point2f> points, following, before, after;
  for (int y = 1; y <= settings.grid_rows; ++y)
    for (int x = 1; x <= settings.grid_columns; ++x)
      points.emplace_back(float(x) * (previous.cols - 1) / (settings.grid_columns + 1),
                          float(y) * (previous.rows - 1) / (settings.grid_rows + 1));
  std::vector<uchar> status;
  std::vector<float> error;
  cv::calcOpticalFlowPyrLK(previous, current, points, following, status, error, {15, 15}, 3,
                           {cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.01});
  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i] && finite(following[i])) {
      before.push_back(points[i]);
      after.push_back(following[i]);
    }
  }
  if (int(before.size()) < settings.min_matches) return {};
  cv::Mat inliers;
  auto matrix = cv::findHomography(before, after, cv::RANSAC, settings.ransac_threshold, inliers);
  if (matrix.empty() || !cv::checkRange(matrix) || std::abs(cv::determinant(matrix)) < 1e-8 ||
      cv::countNonZero(inliers) < settings.min_matches)
    return {};
  double background_motion = 0;
  int count = 0;
  for (size_t i = 0; i < before.size(); ++i) {
    if (inliers.at<uchar>(int(i))) {
      background_motion += cv::norm(after[i] - before[i]);
      ++count;
    }
  }
  return {matrix, background_motion / count};
}

std::vector<cv::Rect> merge_regions(std::vector<cv::Rect> boxes, double distance) {
  std::vector<cv::Rect> merged;
  while (!boxes.empty()) {
    auto region = boxes.front();
    boxes.erase(boxes.begin());
    bool changed = true;
    while (changed) {
      changed = false;
      // Collect the same pass against the current region, then expand (as in numpy).
      std::vector<cv::Rect> remaining;
      auto expanded = region;
      for (const auto &box : boxes) {
        const int dx = std::max({0, region.x - box.br().x, box.x - region.br().x});
        const int dy = std::max({0, region.y - box.br().y, box.y - region.br().y});
        if (std::hypot(dx, dy) < distance) {
          expanded |= box;
          changed = true;
        } else
          remaining.push_back(box);
      }
      region = expanded;
      boxes = std::move(remaining);
    }
    merged.push_back(region);
  }
  return merged;
}

MotionDetector::MotionDetector(const std::string &path, const Settings &settings)
    : classifier_(path, settings), settings_(settings) {}

std::optional<Detection> MotionDetector::detect(const cv::Mat &previous, const cv::Mat &current,
                                                MotionDiagnostics &diagnostics,
                                                std::optional<cv::Point2f> reference) {
  diagnostics = {};
  diagnostics.called = true;
  const auto &s = settings_;
  cv::Mat before, after, blurred;
  // Python numpy slices do not expose parent-image border pixels to OpenCV.
  cv::GaussianBlur(previous, blurred, {11, 11}, 0, 0, cv::BORDER_DEFAULT | cv::BORDER_ISOLATED);
  cv::cvtColor(blurred, before, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(current, blurred, {11, 11}, 0, 0, cv::BORDER_DEFAULT | cv::BORDER_ISOLATED);
  cv::cvtColor(blurred, after, cv::COLOR_BGR2GRAY);
  const auto homography = estimate_homography(before, after, s);
  if (homography.matrix.empty()) return std::nullopt;
  diagnostics.homography_ok = true;
  cv::Mat aligned, valid, difference, binary;
  cv::warpPerspective(before, aligned, homography.matrix, before.size());
  cv::warpPerspective(cv::Mat(before.size(), CV_8U, cv::Scalar(255)), valid, homography.matrix,
                      before.size());
  const auto kernel = cv::Mat::ones(3, 3, CV_8U);
  cv::erode(valid, valid, kernel);
  cv::absdiff(after, before, difference);
  const double threshold = s.difference_threshold + s.light_coefficient * cv::mean(difference)[0] +
                           s.background_coefficient * homography.background_motion;
  cv::absdiff(after, aligned, difference);
  cv::threshold(difference, binary, threshold, 255, cv::THRESH_BINARY);
  binary.setTo(0, valid < 255);
  cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel, {-1, -1}, s.open_iterations);
  cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel, {-1, -1}, s.close_iterations);
  cv::Mat labels, stats, centroids;
  const int components = cv::connectedComponentsWithStats(binary, labels, stats, centroids, 8);
  std::vector<cv::Rect> boxes;
  for (int i = 1; i < components; ++i) {
    if (stats.at<int>(i, cv::CC_STAT_AREA) >= s.min_area)
      boxes.emplace_back(stats.at<int>(i, cv::CC_STAT_LEFT), stats.at<int>(i, cv::CC_STAT_TOP),
                         stats.at<int>(i, cv::CC_STAT_WIDTH), stats.at<int>(i, cv::CC_STAT_HEIGHT));
  }
  const auto regions = merge_regions(boxes, s.merge_distance);
  diagnostics.candidates = int(regions.size());
  std::vector<cv::Rect> proposals;
  std::vector<cv::Mat> patches;
  for (const auto &box : regions) {
    const auto patch = cv::Rect(box.x - 2, box.y - 2, box.width + 4, box.height + 4) &
                       cv::Rect(0, 0, current.cols, current.rows);
    const auto first = aligned(patch).clone(), second = after(patch).clone();
    std::vector<cv::Point2f> corners, following;
    cv::goodFeaturesToTrack(first, corners, 30, 0.15, 3, cv::noArray(), 3);
    if (corners.empty()) continue;
    std::vector<uchar> status;
    std::vector<float> error;
    cv::calcOpticalFlowPyrLK(first, second, corners, following, status, error, {15, 15}, 3,
                             {cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.03});
    std::vector<double> speeds, angles;
    for (size_t i = 0; i < status.size(); ++i) {
      const auto vector = following[i] - corners[i];
      if (status[i] && finite(vector)) {
        speeds.push_back(cv::norm(vector));
        angles.push_back(std::atan2(vector.y, vector.x));
      }
    }
    if (speeds.empty()) continue;
    cv::Scalar speed_mean, speed_std, angle_mean, angle_std;
    cv::meanStdDev(speeds, speed_mean, speed_std);
    cv::meanStdDev(angles, angle_mean, angle_std);
    if (speed_std[0] * speed_std[0] > s.speed_variance || angle_std[0] * angle_std[0] > s.angle_variance ||
        speed_mean[0] < s.min_speed)
      continue;
    proposals.push_back(box);
    cv::Mat resized;
    cv::resize(current(patch), resized, {32, 32});
    patches.push_back(resized);
  }
  diagnostics.motion_pass = int(proposals.size());
  if (patches.empty()) return std::nullopt;
  // Official CNN expects BGR, no normalization other than division by 255.
  const auto blob = cv::dnn::blobFromImages(patches, 1.0 / 255.0, {32, 32}, {}, false, false, CV_32F);
  std::vector<int64_t> shape;
  const auto logits = classifier_.run(blob, shape);
  if (shape != std::vector<int64_t>{int64_t(patches.size()), 2})
    throw std::runtime_error("Expected appearance CNN [N,2] logits");
  std::optional<Detection> best;
  for (size_t i = 0; i < proposals.size(); ++i) {
    const float background = logits[2 * i], mav = logits[2 * i + 1];
    if (!std::isfinite(background) || !std::isfinite(mav) || mav <= background) continue;
    ++diagnostics.cnn_pass;
    const float probability = 1.F / (1.F + std::exp(background - mav));
    Detection detection{cv::Rect2f(proposals[i]), probability};
    if (!best ||
        (reference ? cv::norm(detection.center() - *reference) < cv::norm(best->center() - *reference)
                   : detection.score > best->score))
      best = detection;
  }
  return best;
}

}  // namespace ard_mav
