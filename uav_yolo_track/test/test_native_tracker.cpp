#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

#include "uav_yolo_track/pipeline.hpp"

TEST(NativeTracker, PreservesIDThroughLowConfidenceAndShortOcclusion) {
  const auto settings = yolo_track::default_settings(TRACKER_SHARE);
  yolo_track::NativeTracker tracker(settings.python, settings.worker, settings.tracker_config);
  const auto image = cv::Mat::zeros(480, 640, CV_8UC3);
  auto tracks = tracker.update(image, {{{450, 200, 30, 30}, .9F}});
  ASSERT_EQ(tracks.size(), 1U);
  const int id = tracks[0].id;
  EXPECT_NEAR(tracks[0].box.x, 450, .01);
  // A low-score detection recovers an EXISTING track, never starts a new one.
  tracks = tracker.update(image, {{{451, 200, 30, 30}, .12F}});
  ASSERT_EQ(tracks.size(), 1U);
  EXPECT_EQ(tracks[0].id, id);
  EXPECT_NEAR(tracks[0].confidence, .12F, 1e-6);
  EXPECT_TRUE(tracker.update(image, {}).empty());  // Unobserved Kalman predictions are not published.
  tracks = tracker.update(image, {{{453, 200, 30, 30}, .9F}});
  ASSERT_EQ(tracks.size(), 1U);
  EXPECT_EQ(tracks[0].id, id);
  tracker.reset();
  EXPECT_TRUE(tracker.update(image, {}).empty());
}

TEST(NativeTracker, ReportsMissingOrExitedWorkerInsteadOfHanging) {
  const auto settings = yolo_track::default_settings(TRACKER_SHARE);
  EXPECT_THROW(yolo_track::NativeTracker("/no/such/python", settings.worker, settings.tracker_config),
               std::runtime_error);
  EXPECT_THROW(yolo_track::NativeTracker("/bin/false", settings.worker, settings.tracker_config),
               std::runtime_error);
}

TEST(SearchRegion, UsesLastBoxAndExpandsInsideImageWithoutVelocityPrediction) {
  yolo_track::Settings settings;
  const cv::Rect2f box{610, -30, 20, 20};
  EXPECT_EQ(yolo_track::search_region(box, 2, {640, 480}, settings), cv::Rect(332, 0, 308, 308));
  EXPECT_EQ(yolo_track::search_region(box, 100, {640, 480}, settings), cv::Rect(160, 0, 480, 480));
}

TEST(SingleModel, AlwaysUsesFullFrameWithoutLocalModelOrRegion) {
  auto settings = yolo_track::default_settings(TRACKER_SHARE);
  // A single threshold may be below the unused local default (0.1).
  settings.inference.global_conf = .05F;
  const auto model = std::filesystem::path(ament_index_cpp::get_package_share_directory("uav_ard_mav")) /
                     "models/global_best.onnx";
  yolo_track::Pipeline pipeline(model.string(), settings);
  for (const cv::Size &size : {cv::Size(640, 480), cv::Size(640, 480), cv::Size(320, 240)}) {
    const auto result = pipeline.process(cv::Mat::zeros(size, CV_8UC3));
    EXPECT_EQ(result.mode, "SINGLE");
    EXPECT_EQ(result.next_mode, "SINGLE");
    EXPECT_FALSE(result.roi.has_value());
  }
}
