#include <gtest/gtest.h>

#include "uav_ard_mav/tracker.hpp"

TEST(SearchRegion, GrowsAndShiftsAtImageEdges) {
  ard_mav::Settings s;
  EXPECT_EQ(ard_mav::search_region({639, -30}, 2, {640, 480}, s), cv::Rect(332, 0, 308, 308));
  EXPECT_EQ(ard_mav::search_region({-100, 1000}, 100, {640, 480}, s), cv::Rect(0, 0, 480, 480));
  EXPECT_EQ(ard_mav::search_region({160.5F, 160.5F}, 0, {640, 480}, s), cv::Rect(10, 10, 300, 300));
}

TEST(RelativeVelocity, CameraTranslationIsNotTargetVelocity) {
  ard_mav::RelativeVelocity state({100, 100}, {});
  const cv::Mat transform = (cv::Mat_<double>(3, 3) << 1, 0, 5, 0, 1, -3, 0, 0, 1);
  for (int i = 1; i <= 10; ++i) {
    const auto prediction = state.predict(transform);
    const cv::Point2f observed{100.F + 5 * i, 100.F - 3 * i};
    EXPECT_LT(cv::norm(prediction.first - observed), 1e-5);
    state.correct(observed, prediction.second);
  }
  // A missed frame advances prediction once, without a fabricated measurement.
  EXPECT_LT(cv::norm(state.predict(transform).first - cv::Point2f(155, 67)), 1e-5);
}

TEST(Homography, RecoversCameraTranslationAndRejectsTexturelessFrames) {
  cv::Mat before(240, 320, CV_8U);
  cv::RNG random(42);
  random.fill(before, cv::RNG::UNIFORM, 0, 255);
  cv::GaussianBlur(before, before, {3, 3}, 0);
  const cv::Mat transform = (cv::Mat_<double>(3, 3) << 1, 0, 3, 0, 1, 2, 0, 0, 1);
  cv::Mat after;
  cv::warpPerspective(before, after, transform, before.size());
  const auto result = ard_mav::estimate_homography(before, after, {});
  ASSERT_FALSE(result.matrix.empty());
  EXPECT_NEAR(result.matrix.at<double>(0, 2), 3, 0.3);
  EXPECT_NEAR(result.matrix.at<double>(1, 2), 2, 0.3);
  cv::Mat blank = cv::Mat::zeros(240, 320, CV_8U);
  EXPECT_TRUE(ard_mav::estimate_homography(blank, blank, {}).matrix.empty());
}

TEST(Motion, MergesChainedRegionsAndRejectsUnalignedBlankImages) {
  const auto boxes = ard_mav::merge_regions({{0, 0, 5, 5}, {12, 0, 5, 5}, {24, 0, 5, 5}, {100, 0, 5, 5}}, 10);
  ASSERT_EQ(boxes.size(), 2U);
  EXPECT_EQ(boxes[0], cv::Rect(0, 0, 29, 5));
  ard_mav::MotionDetector motion(std::string(ARD_MAV_MODELS) + "/Net_best.onnx", {});
  ard_mav::MotionDiagnostics diagnostics;
  cv::Mat blank = cv::Mat::zeros(240, 320, CV_8UC3);
  EXPECT_FALSE(motion.detect(blank, blank, diagnostics));
  EXPECT_TRUE(diagnostics.called);
  EXPECT_FALSE(diagnostics.homography_ok);
}

TEST(Tracker, BlankFramesAndResolutionChangesDoNotInventDetections) {
  const std::string models(ARD_MAV_MODELS);
  ard_mav::Tracker tracker(models + "/global_best.onnx", models + "/local_best.onnx",
                           models + "/Net_best.onnx", {});
  for (const auto &size : {cv::Size(640, 480), cv::Size(640, 480), cv::Size(320, 240)}) {
    const auto result = tracker.process(cv::Mat::zeros(size, CV_8UC3));
    EXPECT_FALSE(result.detection);
    EXPECT_EQ(result.mode, "GLOBAL");
    EXPECT_EQ(result.next_mode, "GLOBAL");
  }
  EXPECT_THROW(tracker.process(cv::Mat()), std::invalid_argument);
}
