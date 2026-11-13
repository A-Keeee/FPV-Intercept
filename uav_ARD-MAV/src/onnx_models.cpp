#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "uav_ard_mav/tracker.hpp"

namespace ard_mav {

void Settings::validate() const {
  if (local_full_model != "global" && local_full_model != "local" && local_full_model != "local-multi")
    throw std::invalid_argument("local_full_model must be global, local or local-multi");
  const double values[] = {
      global_conf,      local_conf,           process_noise,     measurement_noise,      initial_covariance,
      ransac_threshold, difference_threshold, light_coefficient, background_coefficient, merge_distance,
      angle_variance,   speed_variance,       min_speed, roi_padding,
      full_frame_enter_ratio, full_frame_exit_ratio, roi_fill_enter_ratio, roi_fill_exit_ratio};
  for (double value : values) {
    if (!std::isfinite(value) || value < 0)
      throw std::invalid_argument("Nonfinite or negative GLAD parameter");
  }
  if (imgsz < 160 || imgsz % 32 || local_conf <= 0 || local_conf > global_conf || global_conf > 1 ||
      crop_size < 1 || roi_growth < 0 || roi_padding < 1 || local_patience < 1 || global_search_interval < 1 ||
      full_frame_exit_ratio <= 0 || full_frame_enter_ratio > 1 ||
      full_frame_exit_ratio >= full_frame_enter_ratio ||
      roi_fill_exit_ratio <= 0 || roi_fill_enter_ratio > 1 ||
      roi_fill_exit_ratio >= roi_fill_enter_ratio ||
      measurement_noise <= 0 || grid_columns < 1 ||
      grid_rows < 1 || min_matches < 4 || ransac_threshold <= 0 || min_area < 1 || open_iterations < 0 ||
      close_iterations < 0 || threads < 1 || device_id < 0 || (provider != "cpu" && provider != "cuda")) {
    throw std::invalid_argument("Invalid GLAD settings (imgsz >= 160 and multiple of 32; provider cpu/cuda)");
  }
}

struct OnnxModel::Impl {
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "ard_mav"};
  Ort::Session session{nullptr};
  std::string input_name, output_name;
  std::vector<int64_t> input_shape;
};

OnnxModel::OnnxModel(const std::string &path, const Settings &settings) : impl_(std::make_unique<Impl>()) {
  settings.validate();
  Ort::SessionOptions options;
  options.SetIntraOpNumThreads(settings.threads);
  options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  if (settings.provider == "cuda") {
    OrtCUDAProviderOptions cuda{};
    cuda.device_id = settings.device_id;
    options.AppendExecutionProvider_CUDA(cuda);
  }
  impl_->session = Ort::Session(impl_->env, path.c_str(), options);
  if (impl_->session.GetInputCount() != 1 || impl_->session.GetOutputCount() != 1)
    throw std::runtime_error("Expected a single-input/single-output ONNX model: " + path);
  Ort::AllocatorWithDefaultOptions allocator;
  impl_->input_name = impl_->session.GetInputNameAllocated(0, allocator).get();
  impl_->output_name = impl_->session.GetOutputNameAllocated(0, allocator).get();
  auto input = impl_->session.GetInputTypeInfo(0);
  auto output = impl_->session.GetOutputTypeInfo(0);
  if (input.GetTensorTypeAndShapeInfo().GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
      output.GetTensorTypeAndShapeInfo().GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
    throw std::runtime_error("Expected float32 ONNX models: " + path);
  impl_->input_shape = input.GetTensorTypeAndShapeInfo().GetShape();
}
OnnxModel::~OnnxModel() = default;

std::vector<float> OnnxModel::run(const cv::Mat &blob, std::vector<int64_t> &shape) {
  if (blob.type() != CV_32F || !blob.isContinuous() || blob.dims != 4 || impl_->input_shape.size() != 4)
    throw std::runtime_error("ONNX input must be a contiguous float32 NCHW tensor");
  std::vector<int64_t> input_shape;
  for (int i = 0; i < blob.dims; ++i) {
    input_shape.push_back(blob.size[i]);
    if (impl_->input_shape[i] > 0 && impl_->input_shape[i] != blob.size[i])
      throw std::runtime_error("ONNX input shape mismatch; re-export with scripts/export_models.py");
  }
  auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  auto tensor = Ort::Value::CreateTensor<float>(memory, const_cast<float *>(blob.ptr<float>()), blob.total(),
                                                input_shape.data(), input_shape.size());
  const char *inputs[] = {impl_->input_name.c_str()}, *outputs[] = {impl_->output_name.c_str()};
  auto values = impl_->session.Run(Ort::RunOptions{nullptr}, inputs, &tensor, 1, outputs, 1);
  auto info = values[0].GetTensorTypeAndShapeInfo();
  shape = info.GetShape();
  const auto *data = values[0].GetTensorData<float>();
  return {data, data + info.GetElementCount()};
}

AppearanceDetector::AppearanceDetector(const std::string &path, const Settings &settings)
    : model_(path, settings), imgsz_(settings.imgsz) {
  // Initialize each detector's execution plan before accepting camera frames.
  // Run the detector directly so warmup cannot change the tracker's state.
  const cv::Mat warmup(imgsz_, imgsz_, CV_8UC3, cv::Scalar::all(114));
  for (int i = 0; i < 3; ++i) detect_all(warmup, 0.0F);
}

std::vector<Detection> AppearanceDetector::detect_all(const cv::Mat &image, float confidence) {
  // Preserve the ROI's aspect ratio, but always pad to the same square tensor.
  // Dynamic rectangular tensors rebuild CUDA execution plans as the ROI grows.
  const double ratio = std::min(double(imgsz_) / image.cols, double(imgsz_) / image.rows);
  const int width = int(std::nearbyint(image.cols * ratio));
  const int height = int(std::nearbyint(image.rows * ratio));
  const int dw = imgsz_ - width, dh = imgsz_ - height;
  const int left = int(std::nearbyint(dw / 2.0 - 0.1)), top = int(std::nearbyint(dh / 2.0 - 0.1));
  cv::Mat resized, padded;
  cv::resize(image, resized, {width, height});
  cv::copyMakeBorder(resized, padded, top, dh - top, left, dw - left, cv::BORDER_CONSTANT, {114, 114, 114});
  auto blob = cv::dnn::blobFromImage(padded, 1.0 / 255.0, {}, {}, true, false, CV_32F);
  std::vector<int64_t> shape;
  const auto output = model_.run(blob, shape);
  if (shape.size() != 3 || shape[0] != 1 || shape[2] != 6)
    throw std::runtime_error("Expected YOLO26 end-to-end [1,N,6] xyxy/score/class output; re-export models");
  // scale_boxes recomputes padding from tensor and original sizes.
  const double gain = std::min(double(padded.rows) / image.rows, double(padded.cols) / image.cols);
  const double pad_x = std::nearbyint((padded.cols - image.cols * gain) / 2 - 0.1);
  const double pad_y = std::nearbyint((padded.rows - image.rows * gain) / 2 - 0.1);
  std::vector<Detection> detections;
  for (int64_t i = 0; i < std::min<int64_t>(100, shape[1]); ++i) {
    const auto *row = output.data() + i * 6;
    if (!std::all_of(row, row + 6, [](float x) { return std::isfinite(x); }) || row[4] <= confidence ||
        row[5] != 0)
      continue;
    const float x1 = std::clamp(float((row[0] - pad_x) / gain), 0.F, float(image.cols));
    const float y1 = std::clamp(float((row[1] - pad_y) / gain), 0.F, float(image.rows));
    const float x2 = std::clamp(float((row[2] - pad_x) / gain), 0.F, float(image.cols));
    const float y2 = std::clamp(float((row[3] - pad_y) / gain), 0.F, float(image.rows));
    if (x2 <= x1 || y2 <= y1) continue;
    detections.push_back({{x1, y1, x2 - x1, y2 - y1}, row[4]});
  }
  return detections;
}

std::optional<Detection> AppearanceDetector::detect(const cv::Mat &image, float confidence,
                                                    std::optional<cv::Point2f> reference) {
  std::optional<Detection> best;
  for (const auto &detection : detect_all(image, confidence)) {
    if (!best || detection.score > best->score ||
        (reference && detection.score == best->score &&
         cv::norm(detection.center() - *reference) < cv::norm(best->center() - *reference)))
      best = detection;
  }
  return best;
}

}  // namespace ard_mav
