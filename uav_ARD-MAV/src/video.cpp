#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

#include "uav_ard_mav/tracker.hpp"

int main(int argc, char **argv) {
  if (argc < 3 || argc > 6) {
    std::cerr << "Usage: ard_mav_video VIDEO MODEL_DIR [MAX_FRAMES=0] [CSV=frames.csv] [PROVIDER=cpu]\n";
    return 1;
  }
  try {
    const std::filesystem::path models(argv[2]);
    ard_mav::Settings settings;
    if (argc > 5) settings.provider = argv[5];
    ard_mav::Tracker tracker((models / "global_best.onnx").string(),
                             (models / "local_multiscale_ft_20261030.onnx").string(),
                             (models / "Net_best.onnx").string(), settings);
    cv::VideoCapture capture(argv[1]);
    if (!capture.isOpened()) throw std::runtime_error("Cannot open video");
    const int maximum = argc > 3 ? std::stoi(argv[3]) : 0;
    if (maximum < 0) throw std::invalid_argument("MAX_FRAMES must be nonnegative");
    std::ofstream csv(argc > 4 ? argv[4] : "frames.csv");
    if (!csv) throw std::runtime_error("Cannot create CSV");
    csv << "frame,mode,next_mode,branch,detected,x1,y1,x2,y2,score,lost_frames,roi_x,roi_y,roi_side,"
           "predicted_x,predicted_y,homography_ok,motion_called,motion_homography_ok,candidates,motion_pass,"
           "cnn_pass\n";
    csv << std::setprecision(9);
    cv::Mat frame;
    int count = 0;
    while ((!maximum || count < maximum) && capture.read(frame)) {
      const auto r = tracker.process(frame);
      csv << ++count << ',' << r.mode << ',' << r.next_mode << ',' << r.branch << ',' << bool(r.detection)
          << ',';
      if (r.detection) {
        const auto &b = r.detection->box;
        csv << b.x << ',' << b.y << ',' << b.x + b.width << ',' << b.y + b.height << ','
            << r.detection->score;
      } else
        csv << ",,,,";
      csv << ',' << r.lost << ',';
      if (r.roi)
        csv << r.roi->x << ',' << r.roi->y << ',' << r.roi->width;
      else
        csv << ",,";
      csv << ',';
      if (r.predicted)
        csv << r.predicted->x << ',' << r.predicted->y;
      else
        csv << ',';
      csv << ',' << r.homography_ok << ',' << r.motion.called << ',' << r.motion.homography_ok << ','
          << r.motion.candidates << ',' << r.motion.motion_pass << ',' << r.motion.cnn_pass << '\n';
    }
    if (count == 0) throw std::runtime_error("No decoded frames");
    std::cout << "Processed " << count << " frames\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
