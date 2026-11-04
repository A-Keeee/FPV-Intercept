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
  ard_mav::Settings settings;
  settings.provider = "cpu";
  ard_mav::MotionDetector motion(std::string(ARD_MAV_MODELS) + "/Net_best.onnx", settings);
  ard_mav::MotionDiagnostics diagnostics;
  cv::Mat blank = cv::Mat::zeros(240, 320, CV_8UC3);
  EXPECT_FALSE(motion.detect(blank, blank, diagnostics));
  EXPECT_TRUE(diagnostics.called);
  EXPECT_FALSE(diagnostics.homography_ok);
}

TEST(Tracker, BlankFramesAndResolutionChangesDoNotInventDetections) {
  const std::string models(ARD_MAV_MODELS);
  ard_mav::Settings settings;
  settings.provider = "cpu";
  ard_mav::Tracker tracker(models + "/global_best.onnx", models + "/local_multiscale_ft_20261030.onnx",
                           models + "/Net_best.onnx", settings);
  const std::vector<cv::Size> sizes{{640, 480}, {640, 480}, {640, 480}, {320, 240}, {320, 240}, {320, 240}};
  const std::vector<std::string> modes{"GLOBAL", "LOCAL_MULTI", "GLOBAL", "GLOBAL", "LOCAL_MULTI", "GLOBAL"};
  for (size_t i = 0; i < sizes.size(); ++i) {
    SCOPED_TRACE(i);
    const auto result = tracker.process(cv::Mat::zeros(sizes[i], CV_8UC3));
    EXPECT_FALSE(result.detection);
    EXPECT_EQ(result.mode, modes[i]);
    EXPECT_EQ(result.next_mode, modes[i] == "GLOBAL" ? "LOCAL_MULTI" : "GLOBAL");
    EXPECT_FALSE(result.roi);
    EXPECT_FALSE(result.predicted);
  }
  // Reset must restart at the global model even if LOCAL_MULTI was scheduled.
  tracker.reset();
  for (const auto &mode : {"GLOBAL", "LOCAL_MULTI"}) {
    const auto result = tracker.process(cv::Mat::zeros(240, 320, CV_8UC3));
    EXPECT_FALSE(result.detection);
    EXPECT_EQ(result.mode, mode);
    EXPECT_EQ(result.next_mode, std::string(mode) == "GLOBAL" ? "LOCAL_MULTI" : "GLOBAL");
  }
  EXPECT_THROW(tracker.process(cv::Mat()), std::invalid_argument);
}

TEST(Tracker, LocalMultiReacquiresWhenGlobalCannotDetectAndResumesRoiTracking) {
  ard_mav::Settings settings;
  settings.provider = "cpu";
  settings.local_patience = 2;
  const std::string fixtures(ARD_MAV_TEST_MODELS);
  ard_mav::Tracker tracker(fixtures + "/global_miss.onnx", fixtures + "/local_brightness.onnx",
                           std::string(ARD_MAV_MODELS) + "/Net_best.onnx", settings);
  const cv::Mat blank = cv::Mat::zeros(480, 640, CV_8UC3);
  const cv::Mat target(480, 640, CV_8UC3, cv::Scalar::all(64));
  EXPECT_EQ(tracker.process(blank).next_mode, "LOCAL_MULTI");
  auto result = tracker.process(target);
  ASSERT_TRUE(result.detection);
  EXPECT_EQ(result.mode, "LOCAL_MULTI");
  EXPECT_EQ(result.next_mode, "LOCAL");

  for (int cycle = 0; cycle < 2; ++cycle) {
    SCOPED_TRACE(cycle);
    result = tracker.process(target);
    ASSERT_TRUE(result.detection);
    EXPECT_EQ(result.mode, "LOCAL");
    ASSERT_TRUE(result.roi);
    EXPECT_GT(result.roi->x, 0);
    for (int missed = 1; missed <= settings.local_patience; ++missed) {
      result = tracker.process(blank);
      EXPECT_FALSE(result.detection);
      EXPECT_EQ(result.mode, "LOCAL");
      EXPECT_EQ(result.lost, missed);
      EXPECT_EQ(result.next_mode, missed == settings.local_patience ? "GLOBAL" : "LOCAL");
    }
    // The global model still misses the visible target; the next frame must
    // use the local model over the whole image, without the abandoned ROI.
    result = tracker.process(target);
    EXPECT_FALSE(result.detection);
    EXPECT_EQ(result.mode, "GLOBAL");
    EXPECT_EQ(result.next_mode, "LOCAL_MULTI");
    result = tracker.process(target);
    ASSERT_TRUE(result.detection);
    EXPECT_EQ(result.mode, "LOCAL_MULTI");
    EXPECT_EQ(result.branch, "LAD");
    EXPECT_EQ(result.next_mode, "LOCAL");
    EXPECT_EQ(result.lost, 0);
    EXPECT_FALSE(result.roi);
    EXPECT_FALSE(result.predicted);
    EXPECT_EQ(result.detection->box, cv::Rect2f(320, 160, 160, 160));
    EXPECT_NEAR(result.detection->score, 64.F / 255, 1e-4);
  }
}
