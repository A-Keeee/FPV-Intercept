#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "uav_yolo_track/pipeline.hpp"

#ifdef UAV_YOLO_SINGLE_MODEL
constexpr bool kSingleModel = true;
#else
constexpr bool kSingleModel = false;
#endif

int main(int argc, char **argv) {
  if (argc < 2 || argc > (kSingleModel ? 6 : 5)) {
    std::cerr << "Usage: " << (kSingleModel ? "yolo_single_track_video" : "yolo_track_video")
              << " VIDEO [MAX_FRAMES=0] [CSV=frames.csv] [PROVIDER=cpu]"
              << (kSingleModel ? " [MODEL=global_best.onnx]" : "") << '\n';
    return 1;
  }
  try {
    auto settings =
        yolo_track::default_settings(ament_index_cpp::get_package_share_directory("uav_yolo_track"));
    if (argc > 4) settings.inference.provider = argv[4];
    const auto models =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("uav_ard_mav")) / "models";
    std::unique_ptr<yolo_track::Pipeline> pipeline;
    if (kSingleModel) {
      const auto model = std::filesystem::path(argc > 5 ? argv[5] : "global_best.onnx");
      pipeline = std::make_unique<yolo_track::Pipeline>(
          (model.is_relative() ? models / model : model).string(), settings);
    } else {
      pipeline = std::make_unique<yolo_track::Pipeline>(
          (models / "global_best.onnx").string(), (models / "local_multiscale_ft_20261030.onnx").string(),
          settings);
    }
    cv::VideoCapture video(argv[1]);
    if (!video.isOpened()) throw std::runtime_error("Cannot open video");
    const int maximum = argc > 2 ? std::stoi(argv[2]) : 0;
    if (maximum < 0) throw std::invalid_argument("MAX_FRAMES must be nonnegative");
    std::ofstream csv(argc > 3 ? argv[3] : "frames.csv");
    if (!csv) throw std::runtime_error("Cannot create CSV");
    csv << "frame,mode,next_mode,id,confidence,x,y,width,height,lost,detections,roi_x,roi_y,roi_side\n"
        << std::setprecision(9);
    int count = 0;
    cv::Mat frame;
    while ((!maximum || count < maximum) && video.read(frame)) {
      const auto r = pipeline->process(frame);
      csv << ++count << ',' << r.mode << ',' << r.next_mode << ',';
      if (r.target) {
        const auto &t = *r.target;
        csv << t.id << ',' << t.confidence << ',' << t.box.x << ',' << t.box.y << ',' << t.box.width << ','
            << t.box.height;
      } else
        csv << "-1,0,,,,";
      csv << ',' << r.lost << ',' << r.detections << ',';
      if (r.roi)
        csv << r.roi->x << ',' << r.roi->y << ',' << r.roi->width;
      else
        csv << ",,";
      csv << '\n';
    }
    if (!count) throw std::runtime_error("No decoded frames");
    std::cout << "Processed " << count << " frames\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
