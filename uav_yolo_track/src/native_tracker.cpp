#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <thread>

#include "uav_yolo_track/pipeline.hpp"

extern char **environ;

namespace yolo_track {
using Clock = std::chrono::steady_clock;

NativeTracker::NativeTracker(std::string python, std::string script, std::string config, int timeout_ms)
    : python_(std::move(python)),
      script_(std::move(script)),
      config_(std::move(config)),
      timeout_ms_(timeout_ms) {
  if (timeout_ms <= 0) throw std::invalid_argument("backend_timeout_ms must be positive");
  start();
}
NativeTracker::~NativeTracker() { stop(); }

void NativeTracker::start() {
  int sockets[2];
  if (socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, sockets))
    throw std::runtime_error("Cannot create tracker socket");
  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_adddup2(&actions, sockets[1], STDIN_FILENO);
  posix_spawn_file_actions_adddup2(&actions, sockets[1], STDOUT_FILENO);
  posix_spawn_file_actions_addclose(&actions, sockets[0]);
  posix_spawn_file_actions_addclose(&actions, sockets[1]);
  std::array<char *, 6> argv{python_.data(), const_cast<char *>("-u"),
                             script_.data(), const_cast<char *>("--config"),
                             config_.data(), nullptr};
  pid_t child;
  const int error = posix_spawnp(&child, python_.c_str(), &actions, nullptr, argv.data(), environ);
  posix_spawn_file_actions_destroy(&actions);
  close(sockets[1]);
  if (error) {
    close(sockets[0]);
    throw std::runtime_error("Cannot start Ultralytics Python: " + std::string(std::strerror(error)));
  }
  pid_ = child;
  socket_ = sockets[0];
  try {
    uint32_t version;
    transfer(&version, sizeof(version), false, Clock::now() + std::chrono::seconds(30));
    if (version != 0x59544B31) throw std::runtime_error("Tracker protocol mismatch");
  } catch (...) {
    stop();
    throw;
  }
}

void NativeTracker::stop() noexcept {
  if (socket_ >= 0) {
    close(socket_);
    socket_ = -1;
  }
  if (pid_ > 0) {
    kill(pid_, SIGTERM);
    const auto deadline = Clock::now() + std::chrono::seconds(2);
    while (true) {
      const auto result = waitpid(pid_, nullptr, WNOHANG);
      if (result == pid_ || (result < 0 && errno != EINTR)) break;
      if (Clock::now() >= deadline) {
        kill(pid_, SIGKILL);
        while (waitpid(pid_, nullptr, 0) < 0 && errno == EINTR) {
        }
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    pid_ = -1;
  }
}

void NativeTracker::transfer(void *data, size_t bytes, bool writing, Clock::time_point deadline) {
  auto *cursor = static_cast<char *>(data);
  while (bytes) {
    const int remaining =
        int(std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count());
    if (remaining <= 0) throw std::runtime_error("Native tracker timed out");
    pollfd descriptor{socket_, short(writing ? POLLOUT : POLLIN), 0};
    const int ready = poll(&descriptor, 1, remaining);
    if (ready < 0 && errno == EINTR) continue;
    if (ready <= 0) throw std::runtime_error("Native tracker did not respond");
    const auto count = writing ? send(socket_, cursor, bytes, MSG_NOSIGNAL | MSG_DONTWAIT)
                               : recv(socket_, cursor, bytes, MSG_DONTWAIT);
    if (count < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;
    if (count <= 0) throw std::runtime_error("Native tracker disconnected; check Python stderr");
    cursor += count;
    bytes -= size_t(count);
  }
}

std::vector<Track> NativeTracker::exchange(const cv::Mat &frame,
                                           const std::vector<ard_mav::Detection> &detections,
                                           bool resetting) {
  try {
    if (socket_ < 0) start();
    const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms_);
    std::array<uint32_t, 5> header{uint32_t(resetting), uint32_t(frame.rows), uint32_t(frame.cols),
                                   uint32_t(detections.size()), uint32_t(frame.total() * frame.elemSize())};
    transfer(header.data(), sizeof(header), true, deadline);
    if (!resetting) {
      std::vector<float> boxes;
      for (const auto &detection : detections) {
        const auto &b = detection.box;
        boxes.insert(boxes.end(), {b.x, b.y, b.x + b.width, b.y + b.height, detection.score, 0});
      }
      transfer(boxes.data(), boxes.size() * sizeof(float), true, deadline);
      transfer(frame.data, frame.total() * frame.elemSize(), true, deadline);
    }
    uint32_t count;
    transfer(&count, sizeof(count), false, deadline);
    if (count > 100) throw std::runtime_error("Invalid tracker result count");
    std::vector<float> rows(size_t(count) * 8);
    transfer(rows.data(), rows.size() * sizeof(float), false, deadline);
    std::vector<Track> tracks;
    for (size_t i = 0; i < count; ++i) {
      const auto *r = rows.data() + i * 8;
      if (!std::all_of(r, r + 8, [](float x) { return std::isfinite(x); }) || r[2] <= r[0] || r[3] <= r[1] ||
          r[4] < 1 || r[4] >= 2147483520.F)
        throw std::runtime_error("Invalid native tracker result");
      tracks.push_back({{r[0], r[1], r[2] - r[0], r[3] - r[1]}, int(r[4]), r[5]});
    }
    return tracks;
  } catch (...) {
    stop();
    throw;
  }
}

std::vector<Track> NativeTracker::update(const cv::Mat &frame,
                                         const std::vector<ard_mav::Detection> &detections) {
  if (frame.empty() || frame.type() != CV_8UC3 || detections.size() > 100 ||
      frame.total() * frame.elemSize() > 256 * 1024 * 1024)
    throw std::invalid_argument("Expected BGR8 frame and at most 100 detections");
  const auto continuous = frame.isContinuous() ? frame : frame.clone();
  return exchange(continuous, detections, false);
}
void NativeTracker::reset() {
  if (socket_ >= 0) exchange({}, {}, true);
}

}  // namespace yolo_track
