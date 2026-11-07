#include <gtest/gtest.h>
#include <cmath>
#include <limits>

#include "uav_ard_mav/tracker.hpp"

TEST(SearchRegion, GrowsAndShiftsAtImageEdges) {
  ard_mav::Settings s;
  EXPECT_EQ(ard_mav::search_region({639, -30}, 2, {640, 480}, s), cv::Rect(332, 0, 308, 308));
  EXPECT_EQ(ard_mav::search_region({-100, 1000}, 100, {640, 480}, s), cv::Rect(0, 0, 640, 480));
  EXPECT_EQ(ard_mav::search_region({160.5F, 160.5F}, 0, {640, 480}, s), cv::Rect(10, 10, 300, 300));
}

TEST(SearchRegion, FitsPaddedTargetsWithIndependentImageBoundsAndMissedFrameGrowth) {
  ard_mav::Settings s;
  EXPECT_EQ(ard_mav::search_region({10, 470}, 0, {640, 480}, s, {420, 100}),
            cv::Rect(0, 180, 504, 300));
  EXPECT_EQ(ard_mav::search_region({10, 470}, 2, {640, 480}, s, {420, 100}),
            cv::Rect(0, 172, 512, 308));
  EXPECT_EQ(ard_mav::search_region({320, 240}, 0, {640, 480}, s, {600, 400}),
            cv::Rect(0, 0, 640, 480));
  EXPECT_EQ(ard_mav::search_region({470, 10}, 0, {480, 640}, s, {100, 520}),
            cv::Rect(180, 0, 300, 624));
  // With padding disabled, the crop still cannot be smaller than the box.
  s.roi_padding = 1.0;
  EXPECT_EQ(ard_mav::search_region({320, 240}, 0, {640, 480}, s, {600, 400}).size(),
            cv::Size(600, 400));
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
  settings.global_search_interval = 3;
  ard_mav::Tracker tracker(models + "/global_best.onnx", models + "/local_multiscale_ft_20261030.onnx",
                           models + "/Net_best.onnx", settings);
  const std::vector<cv::Size> sizes{{640, 480}, {640, 480}, {640, 480}, {320, 240}, {320, 240}, {320, 240}};
  const std::vector<std::string> modes{"LOCAL_MULTI", "LOCAL_MULTI", "GLOBAL", "LOCAL_MULTI", "LOCAL_MULTI", "GLOBAL"};
  const std::vector<std::string> next{"LOCAL_MULTI", "GLOBAL", "LOCAL_MULTI", "LOCAL_MULTI", "GLOBAL", "LOCAL_MULTI"};
  for (size_t i = 0; i < sizes.size(); ++i) {
    SCOPED_TRACE(i);
    const auto result = tracker.process(cv::Mat::zeros(sizes[i], CV_8UC3));
    EXPECT_FALSE(result.detection);
    EXPECT_EQ(result.mode, modes[i]);
    EXPECT_EQ(result.next_mode, next[i]);
    EXPECT_FALSE(result.roi);
    EXPECT_FALSE(result.predicted);
  }
  // Explicit reset and resolution changes both restart local-first search.
  tracker.reset();
  for (const auto &next_mode : {"LOCAL_MULTI", "GLOBAL"}) {
    const auto result = tracker.process(cv::Mat::zeros(240, 320, CV_8UC3));
    EXPECT_FALSE(result.detection);
    EXPECT_EQ(result.mode, "LOCAL_MULTI");
    EXPECT_EQ(result.next_mode, next_mode);
  }
  EXPECT_THROW(tracker.process(cv::Mat()), std::invalid_argument);
}

TEST(Tracker, LocalMultiReacquiresWhenGlobalCannotDetectAndResumesRoiTracking) {
  ard_mav::Settings settings;
  settings.provider = "cpu";
  settings.local_patience = 2;
  // This test exercises ROI loss/reacquisition independently of large-target switching.
  settings.full_frame_enter_ratio = 0.8;
  settings.full_frame_exit_ratio = 0.7;
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
      EXPECT_EQ(result.next_mode, missed == settings.local_patience ? "LOCAL_MULTI" : "LOCAL");
    }
    // Reacquire immediately with the local model on the full frame, rather
    // than spending the first recovery frame on the known-missing global model.
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

TEST(Tracker, LargeGlobalAndLocalDetectionsUseFullFrameLocalWithHysteresis) {
  const std::string fixtures(ARD_MAV_TEST_MODELS);
  for (bool start_global : {false, true}) {
    SCOPED_TRACE(start_global);
    ard_mav::Settings settings;
    settings.provider = "cpu";
    settings.global_search_interval = start_global ? 1 : 10;
    const auto global = fixtures + (start_global ? "/scalable_brightness.onnx" : "/global_miss.onnx");
    ard_mav::Tracker tracker(global, fixtures + "/scalable_brightness.onnx",
                             std::string(ARD_MAV_MODELS) + "/Net_best.onnx", settings);
    auto result = tracker.process(cv::Mat(480, 640, CV_8UC3, cv::Scalar::all(128)));
    ASSERT_TRUE(result.detection);
    EXPECT_EQ(result.mode, start_global ? "GLOBAL" : "LOCAL_MULTI");
    EXPECT_EQ(result.branch, start_global ? "GAD" : "LAD");
    EXPECT_EQ(result.next_mode, "LOCAL_FULL");
    const auto initial_size = result.detection->box.size();
    // Between the enter/exit thresholds the tracker must stay full-frame,
    // including when the search scheduler would otherwise use global only.
    for (int i = 0; i < 4; ++i) {
      result = tracker.process(cv::Mat(480, 640, CV_8UC3, cv::Scalar::all(36)));
      ASSERT_TRUE(result.detection);
      EXPECT_EQ(result.mode, "LOCAL_FULL");
      EXPECT_EQ(result.branch, "LAD");
      EXPECT_EQ(result.next_mode, "LOCAL_FULL");
      EXPECT_FALSE(result.roi);
      EXPECT_GT(result.target_fov_ratio, settings.full_frame_exit_ratio);
      EXPECT_LT(result.target_fov_ratio, settings.full_frame_enter_ratio);
      EXPECT_EQ(result.lost, 0);
    }
    result = tracker.process(cv::Mat(480, 640, CV_8UC3, cv::Scalar::all(24)));
    ASSERT_TRUE(result.detection);
    EXPECT_EQ(result.mode, "LOCAL_FULL");
    EXPECT_FALSE(result.roi);
    EXPECT_EQ(result.next_mode, "LOCAL");
    result = tracker.process(cv::Mat(480, 640, CV_8UC3, cv::Scalar::all(24)));
    ASSERT_TRUE(result.detection);
    EXPECT_EQ(result.mode, "LOCAL");
    ASSERT_TRUE(result.roi);
    // Global measurements remain a size floor for this track, even after a
    // smaller full-frame local detection. Local-only tracks can shrink again.
    const int minimum = start_global ? int(std::ceil(initial_size.width * settings.roi_padding)) :
                                      settings.crop_size;
    EXPECT_EQ(result.roi->size(), cv::Size(minimum, minimum));
  }
}

TEST(Tracker, DisabledLocalFullKeepsPaddedRoiAfterGlobalOrLocalAcquisitionAndClearsSizeOnLoss) {
  const std::string fixtures(ARD_MAV_TEST_MODELS);
  for (bool start_global : {false, true}) {
    for (int brightness : {128, 255}) {
      SCOPED_TRACE(start_global);
      SCOPED_TRACE(brightness);
      ard_mav::Settings settings;
      settings.provider = "cpu";
      settings.enable_local_full = false;
      settings.local_patience = 2;
      settings.global_conf = 0.3F;
      settings.global_search_interval = start_global ? 1 : 10;
      const auto global = fixtures + (start_global ? "/scalable_brightness.onnx" : "/global_miss.onnx");
      ard_mav::Tracker tracker(global, fixtures + "/scalable_brightness.onnx",
                               std::string(ARD_MAV_MODELS) + "/Net_best.onnx", settings);
      auto result = tracker.process(cv::Mat(480, 640, CV_8UC3, cv::Scalar::all(brightness)));
      ASSERT_TRUE(result.detection);
      EXPECT_EQ(result.mode, start_global ? "GLOBAL" : "LOCAL_MULTI");
      EXPECT_EQ(result.next_mode, "LOCAL");
      const auto initial_box = result.detection->box;
      const cv::Size minimum{
          std::min(640, int(std::ceil(initial_box.width * settings.roi_padding))),
          std::min(480, int(std::ceil(initial_box.height * settings.roi_padding)))};
      const cv::Mat small(480, 640, CV_8UC3, cv::Scalar::all(24));
      for (int i = 0; i < 4; ++i) {
        result = tracker.process(small);
        ASSERT_TRUE(result.detection);
        ASSERT_TRUE(result.roi);
        EXPECT_EQ(result.mode, "LOCAL");
        EXPECT_EQ(result.next_mode, "LOCAL");
        EXPECT_EQ(result.branch, "LAD");
        EXPECT_GE(result.roi->width, minimum.width);
        EXPECT_GE(result.roi->height, minimum.height);
        EXPECT_EQ(*result.roi & cv::Rect(0, 0, 640, 480), *result.roi);
      }
      for (int missed = 1; missed <= settings.local_patience; ++missed) {
        result = tracker.process(cv::Mat::zeros(480, 640, CV_8UC3));
        ASSERT_TRUE(result.roi);
        EXPECT_GE(result.roi->width, minimum.width);
        EXPECT_GE(result.roi->height, minimum.height);
        EXPECT_EQ(result.mode, "LOCAL");
        EXPECT_EQ(result.next_mode, missed == settings.local_patience ?
                  (start_global ? "GLOBAL" : "LOCAL_MULTI") : "LOCAL");
      }
      result = tracker.process(small);
      ASSERT_TRUE(result.detection);
      EXPECT_FALSE(result.roi);
      result = tracker.process(small);
      ASSERT_TRUE(result.roi);
      EXPECT_EQ(result.roi->size(), cv::Size(300, 300));
    }
  }
}

TEST(Tracker, DisabledLocalFullExpandsRoiForGrowingLocalDetectionsAndExplicitResetClearsSize) {
  ard_mav::Settings settings;
  settings.provider = "cpu";
  settings.enable_local_full = false;
  const std::string fixture = std::string(ARD_MAV_TEST_MODELS) + "/scalable_brightness.onnx";
  ard_mav::Tracker tracker(fixture, fixture, std::string(ARD_MAV_MODELS) + "/Net_best.onnx", settings);
  const cv::Mat small(480, 640, CV_8UC3, cv::Scalar::all(24));
  ASSERT_TRUE(tracker.process(small).detection);
  auto result = tracker.process(cv::Mat(480, 640, CV_8UC3, cv::Scalar::all(255)));
  ASSERT_TRUE(result.detection);
  ASSERT_TRUE(result.roi);
  EXPECT_EQ(result.roi->size(), cv::Size(300, 300));
  EXPECT_EQ(result.next_mode, "LOCAL");
  const auto grown_box = result.detection->box;
  result = tracker.process(small);
  ASSERT_TRUE(result.roi);
  EXPECT_GE(result.roi->width, std::ceil(grown_box.width * settings.roi_padding));
  EXPECT_GE(result.roi->height, std::ceil(grown_box.height * settings.roi_padding));
  EXPECT_GT(result.roi->width, 300);
  tracker.reset();
  EXPECT_FALSE(tracker.process(small).roi);
  result = tracker.process(small);
  ASSERT_TRUE(result.roi);
  EXPECT_EQ(result.roi->size(), cv::Size(300, 300));
  ASSERT_TRUE(tracker.process(cv::Mat(480, 640, CV_8UC3, cv::Scalar::all(255))).detection);
  const cv::Mat resized_small(240, 320, CV_8UC3, cv::Scalar::all(24));
  EXPECT_FALSE(tracker.process(resized_small).roi);
  result = tracker.process(resized_small);
  ASSERT_TRUE(result.roi);
  EXPECT_EQ(result.roi->size(), cv::Size(300, 240));
}

TEST(Tracker, GrowingRoiDetectionSwitchesToFullFrameBeforeCroppingTarget) {
  ard_mav::Settings settings;
  settings.provider = "cpu";
  // Make only the ROI-fill guard trigger, independently of FOV occupancy.
  settings.full_frame_enter_ratio = 0.8;
  settings.full_frame_exit_ratio = 0.7;
  const std::string fixtures(ARD_MAV_TEST_MODELS);
  ard_mav::Tracker tracker(fixtures + "/global_miss.onnx", fixtures + "/scalable_brightness.onnx",
                           std::string(ARD_MAV_MODELS) + "/Net_best.onnx", settings);
  auto result = tracker.process(cv::Mat(480, 640, CV_8UC3, cv::Scalar::all(24)));
  ASSERT_TRUE(result.detection);
  EXPECT_EQ(result.next_mode, "LOCAL");
  result = tracker.process(cv::Mat(480, 640, CV_8UC3, cv::Scalar::all(255)));
  ASSERT_TRUE(result.detection);
  ASSERT_TRUE(result.roi);
  EXPECT_EQ(result.mode, "LOCAL");
  EXPECT_LT(result.target_fov_ratio, settings.full_frame_enter_ratio);
  EXPECT_GT(result.target_roi_fill, settings.roi_fill_enter_ratio);
  EXPECT_DOUBLE_EQ(result.target_fov_ratio,
      std::max(double(result.detection->box.width) / 640, double(result.detection->box.height) / 480));
  EXPECT_EQ(result.next_mode, "LOCAL_FULL");
  result = tracker.process(cv::Mat(480, 640, CV_8UC3, cv::Scalar::all(255)));
  ASSERT_TRUE(result.detection);
  EXPECT_EQ(result.mode, "LOCAL_FULL");
  EXPECT_EQ(result.branch, "LAD");
  EXPECT_FALSE(result.roi);
  EXPECT_GT(result.detection->box.width, settings.crop_size);
}

TEST(Tracker, LocalFullModelSelectionUsesIndependentWeightsAndRestoresRoiAndSearchModels) {
  const std::string fixtures(ARD_MAV_TEST_MODELS);
  for (const std::string selection : {"global", "local", "local-multi"}) {
    SCOPED_TRACE(selection);
    ard_mav::Settings settings;
    settings.provider = "cpu";
    settings.local_full_model = selection;
    settings.global_conf = 0.3F;
    settings.global_search_interval = 1;
    settings.local_patience = 2;
    // Three distinguishable models: global fixed box, ROI always misses,
    // and the independent full-frame model reports a brightness-scaled box.
    ard_mav::Tracker tracker(fixtures + "/local_brightness.onnx", fixtures + "/global_miss.onnx",
                             std::string(ARD_MAV_MODELS) + "/Net_best.onnx", settings,
                             fixtures + "/scalable_brightness.onnx");
    const cv::Mat large(480, 640, CV_8UC3, cv::Scalar::all(128));
    auto result = tracker.process(large);
    ASSERT_TRUE(result.detection);
    EXPECT_EQ(result.mode, "GLOBAL");
    EXPECT_EQ(result.detection->box.width, 160);
    EXPECT_EQ(result.next_mode, "LOCAL_FULL");
    result = tracker.process(cv::Mat(480, 640, CV_8UC3, cv::Scalar::all(36)));
    ASSERT_TRUE(result.detection);
    EXPECT_EQ(result.mode, "LOCAL_FULL");
    EXPECT_EQ(result.next_mode, "LOCAL_FULL");
    EXPECT_EQ(result.branch, selection == "global" ? "GAD" : "LAD");
    EXPECT_FALSE(result.roi);
    EXPECT_NEAR(result.detection->box.width, 600.0 * 36 / 255, 0.001);
    result = tracker.process(cv::Mat(480, 640, CV_8UC3, cv::Scalar::all(24)));
    ASSERT_TRUE(result.detection);
    EXPECT_EQ(result.mode, "LOCAL_FULL");
    EXPECT_EQ(result.next_mode, "LOCAL");
    // Returning to ROI must restore the original ROI model, not the one
    // selected exclusively for full-frame tracking.
    for (int missed = 1; missed <= settings.local_patience; ++missed) {
      result = tracker.process(large);
      EXPECT_EQ(result.mode, "LOCAL");
      EXPECT_TRUE(result.roi);
      EXPECT_FALSE(result.detection);
      EXPECT_EQ(result.lost, missed);
    }
    EXPECT_EQ(result.next_mode, "GLOBAL");
    result = tracker.process(large);
    ASSERT_TRUE(result.detection);
    EXPECT_EQ(result.mode, "GLOBAL");
    EXPECT_EQ(result.detection->box.width, 160);
  }
}

TEST(Tracker, LocalFullGlobalSelectionUsesGlobalConfidenceAndLocalSelectionsUseLocalConfidence) {
  const std::string fixtures(ARD_MAV_TEST_MODELS);
  for (const std::string selection : {"global", "local", "local-multi"}) {
    SCOPED_TRACE(selection);
    ard_mav::Settings settings;
    settings.provider = "cpu";
    settings.local_full_model = selection;
    settings.global_search_interval = 1;
    ard_mav::Tracker tracker(fixtures + "/scalable_brightness.onnx", fixtures + "/scalable_brightness.onnx",
                             std::string(ARD_MAV_MODELS) + "/Net_best.onnx", settings,
                             fixtures + "/local_brightness.onnx");
    EXPECT_EQ(tracker.process(cv::Mat(480, 640, CV_8UC3, cv::Scalar::all(128))).next_mode, "LOCAL_FULL");
    // The independent model's 0.25 score is below global_conf=0.5 but
    // above local_conf=0.1. A mistakenly reused scalable model would score 0.95.
    const auto result = tracker.process(cv::Mat(480, 640, CV_8UC3, cv::Scalar::all(64)));
    EXPECT_EQ(result.mode, "LOCAL_FULL");
    EXPECT_EQ(bool(result.detection), selection != "global");
    if (result.detection) {
      EXPECT_EQ(result.detection->box.width, 160);
      EXPECT_EQ(result.branch, "LAD");
      EXPECT_NEAR(result.detection->score, 64.0 / 255, 0.0001);
    }
  }
}

TEST(Tracker, LocalFullCanReuseGlobalModelWhenNoSeparatePathIsSupplied) {
  const std::string fixtures(ARD_MAV_TEST_MODELS);
  ard_mav::Settings settings;
  settings.provider = "cpu";
  settings.local_full_model = "global";
  settings.global_search_interval = 1;
  ard_mav::Tracker tracker(fixtures + "/scalable_brightness.onnx", fixtures + "/global_miss.onnx",
                           std::string(ARD_MAV_MODELS) + "/Net_best.onnx", settings);
  EXPECT_EQ(tracker.process(cv::Mat(480, 640, CV_8UC3, cv::Scalar::all(128))).next_mode, "LOCAL_FULL");
  const auto result = tracker.process(cv::Mat(480, 640, CV_8UC3, cv::Scalar::all(36)));
  ASSERT_TRUE(result.detection);
  EXPECT_EQ(result.mode, "LOCAL_FULL");
  EXPECT_EQ(result.branch, "GAD");
  EXPECT_FALSE(result.roi);
  EXPECT_NEAR(result.detection->box.width, 600.0 * 36 / 255, 0.001);
}

TEST(Tracker, DisabledLocalFullDoesNotLoadItsSeparateWeights) {
  const std::string fixtures(ARD_MAV_TEST_MODELS);
  ard_mav::Settings settings;
  settings.provider = "cpu";
  settings.enable_local_full = false;
  settings.local_full_model = "local";
  settings.global_search_interval = 1;
  ard_mav::Tracker tracker(fixtures + "/scalable_brightness.onnx", fixtures + "/scalable_brightness.onnx",
                           std::string(ARD_MAV_MODELS) + "/Net_best.onnx", settings,
                           fixtures + "/does_not_exist.onnx");
  const cv::Mat target(480, 640, CV_8UC3, cv::Scalar::all(128));
  EXPECT_EQ(tracker.process(target).next_mode, "LOCAL");
  const auto result = tracker.process(target);
  EXPECT_EQ(result.mode, "LOCAL");
  EXPECT_TRUE(result.roi);
  EXPECT_TRUE(result.detection);
}

TEST(Tracker, LargeTargetLossAndResetClearFullFrameTrackingState) {
  ard_mav::Settings settings;
  settings.provider = "cpu";
  settings.local_patience = 2;
  settings.global_search_interval = 1;
  const std::string fixture = std::string(ARD_MAV_TEST_MODELS) + "/scalable_brightness.onnx";
  ard_mav::Tracker tracker(fixture, fixture, std::string(ARD_MAV_MODELS) + "/Net_best.onnx", settings);
  const cv::Mat large(480, 640, CV_8UC3, cv::Scalar::all(128));
  const cv::Mat blank = cv::Mat::zeros(480, 640, CV_8UC3);
  EXPECT_EQ(tracker.process(large).next_mode, "LOCAL_FULL");
  for (int missed = 1; missed <= settings.local_patience; ++missed) {
    const auto result = tracker.process(blank);
    EXPECT_FALSE(result.detection);
    EXPECT_FALSE(result.roi);
    EXPECT_EQ(result.mode, "LOCAL_FULL");
    EXPECT_EQ(result.lost, missed);
    EXPECT_EQ(result.next_mode, missed == settings.local_patience ? "GLOBAL" : "LOCAL_FULL");
  }
  EXPECT_EQ(tracker.process(large).mode, "GLOBAL");
  tracker.reset();
  auto result = tracker.process(blank);
  EXPECT_EQ(result.mode, "GLOBAL");
  EXPECT_FALSE(result.predicted);
  EXPECT_EQ(tracker.process(large).next_mode, "LOCAL_FULL");
  result = tracker.process(cv::Mat::zeros(240, 320, CV_8UC3));
  EXPECT_EQ(result.mode, "GLOBAL");
  EXPECT_FALSE(result.predicted);
}

TEST(Settings, LargeTargetThresholdsMustBeFiniteOrderedRatios) {
  ard_mav::Settings settings;
  settings.full_frame_exit_ratio = settings.full_frame_enter_ratio;
  EXPECT_THROW(settings.validate(), std::invalid_argument);
  settings.full_frame_exit_ratio = 0.15;
  settings.roi_fill_enter_ratio = 1.1;
  EXPECT_THROW(settings.validate(), std::invalid_argument);
  settings.roi_fill_enter_ratio = 0.8;
  settings.roi_fill_exit_ratio = 0;
  EXPECT_THROW(settings.validate(), std::invalid_argument);
  settings.roi_fill_exit_ratio = 0.6;
  settings.full_frame_enter_ratio = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(settings.validate(), std::invalid_argument);
}

TEST(Settings, RoiPaddingCannotReduceTargetSizeOrBeNonfinite) {
  ard_mav::Settings settings;
  for (double padding : {0.99, std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity()}) {
    settings.roi_padding = padding;
    EXPECT_THROW(settings.validate(), std::invalid_argument);
  }
  settings.roi_padding = 1.0;
  EXPECT_NO_THROW(settings.validate());
}

TEST(Settings, LocalFullModelMustBeOneOfThreeSupportedChoices) {
  ard_mav::Settings settings;
  for (const std::string selection : {"global", "local", "local-multi"}) {
    settings.local_full_model = selection;
    EXPECT_NO_THROW(settings.validate());
  }
  for (const std::string selection : {"", "GLOBAL", "local_multi", "other"}) {
    settings.local_full_model = selection;
    EXPECT_THROW(settings.validate(), std::invalid_argument);
  }
}

TEST(Tracker, GlobalSearchIsPeriodicAndMotionFallbackCanBeDisabled) {
  ard_mav::Settings settings;
  settings.provider = "cpu";
  settings.global_search_interval = 4;
  settings.motion_fallback = false;
  const std::string fixtures(ARD_MAV_TEST_MODELS);
  ard_mav::Tracker tracker(fixtures + "/global_miss.onnx", fixtures + "/local_brightness.onnx",
                           std::string(ARD_MAV_MODELS) + "/Net_best.onnx", settings);
  const cv::Mat blank = cv::Mat::zeros(480, 640, CV_8UC3);
  for (int i = 1; i <= 20; ++i) {
    SCOPED_TRACE(i);
    const auto result = tracker.process(blank);
    EXPECT_FALSE(result.detection);
    EXPECT_FALSE(result.motion.called);
    EXPECT_EQ(result.mode, i % 4 == 0 ? "GLOBAL" : "LOCAL_MULTI");
    EXPECT_EQ(result.next_mode, (i + 1) % 4 == 0 ? "GLOBAL" : "LOCAL_MULTI");
  }
  settings.global_search_interval = 1;
  ard_mav::Tracker global_only(fixtures + "/global_miss.onnx", fixtures + "/local_brightness.onnx",
                               std::string(ARD_MAV_MODELS) + "/Net_best.onnx", settings);
  for (int i = 0; i < 3; ++i) {
    const auto result = global_only.process(blank);
    EXPECT_EQ(result.mode, "GLOBAL");
    EXPECT_EQ(result.next_mode, "GLOBAL");
  }
  settings.global_search_interval = 0;
  EXPECT_THROW(settings.validate(), std::invalid_argument);
}
