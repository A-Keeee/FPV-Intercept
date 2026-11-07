"""Exercise the real node in an isolated ROS domain, with synthetic PX4 inputs."""

import math
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import HoverThrustEstimate, SensorGps, TrajectorySetpoint, VehicleLocalPosition, VehicleOdometry
from uav_common_msg.msg import RectMsg


class Controller:
    def __init__(self, directory, csv_path, record_csv=False, fallback_png=False, parameters=None):
        self.directory = directory
        self.csv_path = csv_path
        self.record_csv = record_csv
        self.fallback_png = fallback_png
        self.close_target = False
        self.detection = 'valid'
        self.parameters = parameters or {}
        self.box = (940, 520, 40, 40)
        self.commands = []
        self.follow_commands = False
        self.velocity = [0., 0., 0.]

    def __enter__(self):
        # Isolate all arm/offboard publications from the user's PX4 session.
        os.environ['ROS_DOMAIN_ID'] = str(140 + os.getpid() % 50)
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        rclpy.init()
        self.node = Node('statistics_regression')
        self.node.create_subscription(TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint',
                                      self.on_command, 2000)
        self.publishers = {
            'position': self.node.create_publisher(
                VehicleLocalPosition, '/px4_1/fmu/out/vehicle_local_position', qos_profile_sensor_data),
            'attitude': self.node.create_publisher(
                VehicleOdometry, '/px4_1/fmu/out/vehicle_odometry', qos_profile_sensor_data),
            'hover': self.node.create_publisher(
                HoverThrustEstimate, '/px4_1/fmu/out/hover_thrust_estimate', qos_profile_sensor_data),
            'self_gps': self.node.create_publisher(
                SensorGps, '/px4_1/fmu/out/vehicle_gps_position', qos_profile_sensor_data),
            'target_gps': self.node.create_publisher(
                SensorGps, '/px4_2/fmu/out/vehicle_gps_position', qos_profile_sensor_data),
            'detection': self.node.create_publisher(RectMsg, '/camera_detect_result', 10),
        }
        self.node.create_timer(0.01, self.publish_state)
        self.node.create_timer(1 / 60, self.publish_detection)
        self.output_path = self.directory / 'console.log'
        self.output_file = self.output_path.open('w')
        environment = os.environ.copy()
        environment['ROS_LOG_DIR'] = str(self.directory / 'ros_logs')
        command = [
            os.environ['RL_NODE_EXECUTABLE'], '--ros-args',
            '-p', f'csv_path:={self.csv_path}',
            '-p', f'record_csv:={str(self.record_csv).lower()}',
            '-p', 'bench_test:=true',
            '-p', f'fallback_png:={str(self.fallback_png).lower()}',
            '-p', f"model_path:={Path(__file__).resolve().parents[1] / 'models/policy.onnx'}",
        ]
        for name, value in self.parameters.items():
            command.extend(['-p', f'{name}:={value}'])
        self.process = subprocess.Popen(command, stdout=self.output_file, stderr=subprocess.STDOUT, env=environment)
        return self

    def __exit__(self, *args):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        self.output_file.close()
        self.node.destroy_node()
        rclpy.shutdown()

    @property
    def output(self):
        return self.output_path.read_text()

    def publish_state(self):
        position = VehicleLocalPosition()
        position.z = -6.0
        position.vx, position.vy, position.vz = self.velocity
        self.publishers['position'].publish(position)
        attitude = VehicleOdometry()
        attitude.q = [1., 0., 0., 0.]
        self.publishers['attitude'].publish(attitude)
        hover = HoverThrustEstimate()
        hover.hover_thrust = 0.5
        hover.hover_thrust_var = 0.0
        self.publishers['hover'].publish(hover)
        for name, distance in [('self_gps', 0.0), ('target_gps', 0.3 if self.close_target else 20.0)]:
            gps = SensorGps()
            gps.latitude_deg = 47.0
            gps.longitude_deg = 8.0 + distance / (111320.0 * math.cos(math.radians(47.0)))
            gps.altitude_msl_m = 500.0
            self.publishers[name].publish(gps)

    def publish_detection(self):
        if self.detection == 'silent':
            return
        detection = RectMsg()
        detection.header.stamp = self.node.get_clock().now().to_msg()
        if self.detection == 'valid':
            detection.x, detection.y, detection.width, detection.height = self.box
        else:
            detection.x = detection.y = detection.width = detection.height = -1
        self.publishers['detection'].publish(detection)

    def on_command(self, message):
        if all(math.isnan(value) for value in message.position):
            self.commands.append((time.monotonic(), message))
            if self.follow_commands:
                self.velocity = [float(value) for value in message.velocity]

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.005)
            assert self.process.poll() is None, self.output

    def wait_for(self, text, timeout=6.0):
        deadline = time.monotonic() + timeout
        while text not in self.output:
            self.spin_for(0.02)
            assert time.monotonic() < deadline, self.output[-6000:]

    def approach_and_check_success(self):
        self.close_target = True
        self.wait_for('拦截完成，悬停中', timeout=1.5)
        self.spin_for(0.15)  # let the recording timer export the DONE sample
        assert '[STATS] event=hit state=DONE' in self.output


def test_log_only_preserves_existing_csv_and_writes_session_log(tmp_path):
    csv_path = tmp_path / 'existing.csv'
    csv_path.write_text('previous session must remain unchanged\n')
    csv_path.chmod(0o444)
    with Controller(tmp_path, csv_path) as controller:
        controller.wait_for('进入 INTERCEPT')
        controller.spin_for(0.2)
        assert 'ONNX 模型已加载' in controller.output
        assert 'cmd_source=policy' in controller.output
        controller.approach_and_check_success()
    assert csv_path.read_text() == 'previous session must remain unchanged\n'
    assert 'csv=disabled' in controller.output
    assert 'event=summary hit=true' in controller.output
    log_files = list((tmp_path / 'ros_logs').glob('uav_rl_guidance_*.log'))
    assert log_files
    assert any('event=hit' in path.read_text() for path in log_files)


def test_unwritable_csv_cannot_disable_success(tmp_path):
    csv_path = tmp_path / 'readonly.csv'
    csv_path.write_text('read only\n')
    csv_path.chmod(0o444)
    with Controller(tmp_path, csv_path, record_csv=True) as controller:
        controller.wait_for('进入 INTERCEPT')
        controller.spin_for(0.2)
        controller.approach_and_check_success()
    assert controller.output.count('CSV disabled: cannot open') == 1
    assert csv_path.read_text() == 'read only\n'


@pytest.mark.parametrize('detection', ['invalid', 'silent'])
def test_success_evaluation_continues_after_visual_loss(tmp_path, detection):
    csv_path = tmp_path / 'unused.csv'
    with Controller(tmp_path, csv_path) as controller:
        controller.wait_for('进入 INTERCEPT')
        controller.detection = detection
        controller.wait_for('[TRACK_LOST]', timeout=3.0)
        controller.approach_and_check_success()
    assert not csv_path.exists()
    assert 'event=summary hit=true' in controller.output


def test_optional_csv_exports_done_and_hit_summary(tmp_path):
    csv_path = tmp_path / 'optional.csv'
    with Controller(tmp_path, csv_path, record_csv=True) as controller:
        controller.wait_for('进入 INTERCEPT')
        controller.spin_for(0.4)
        controller.approach_and_check_success()
    rows = [line for line in csv_path.read_text().splitlines() if line and not line.startswith('#')]
    assert rows[0].startswith('time_s,self_x,')
    assert rows[-1].endswith(',4')
    assert len(rows) <= 15  # the sink is sampled at 10 Hz, not the 50 Hz control rate
    assert '# 命中时刻(s),' in csv_path.read_text()


def test_png_baseline_evaluates_success_during_short_coast(tmp_path):
    csv_path = tmp_path / 'baseline_readonly.csv'
    csv_path.write_text('keep previous data\n')
    csv_path.chmod(0o444)
    with Controller(tmp_path, csv_path, record_csv=True, fallback_png=True) as controller:
        controller.wait_for('进入 INTERCEPT')
        controller.spin_for(0.2)
        controller.detection = 'invalid'
        controller.spin_for(0.1)  # fewer than coast_thresh invalid messages
        controller.approach_and_check_success()
    assert 'fallback_png=true' in controller.output
    assert 'cmd_source=png' in controller.output
    assert csv_path.read_text() == 'keep previous data\n'


@pytest.mark.skipif(not Path('/dev/full').exists(), reason='requires Linux /dev/full')
def test_csv_write_failure_cannot_disable_success(tmp_path):
    with Controller(tmp_path, Path('/dev/full'), record_csv=True) as controller:
        controller.wait_for('进入 INTERCEPT')
        controller.spin_for(0.2)
        controller.approach_and_check_success()
    assert controller.output.count('CSV disabled: write failed') == 1
    assert 'event=summary hit=true' in controller.output


def speed(message):
    return math.sqrt(sum(float(value) ** 2 for value in message.velocity))


def assert_command_limits(commands, max_speed=3.0):
    for (_, previous), (_, current) in zip(commands, commands[1:]):
        dt = (current.timestamp - previous.timestamp) / 1e6
        assert dt > 0
        dt = min(dt, 0.05)
        xy_change = math.hypot(float(current.velocity[0] - previous.velocity[0]),
                               float(current.velocity[1] - previous.velocity[1]))
        assert xy_change <= 0.8 * dt + 2e-5
        assert abs(float(current.velocity[2] - previous.velocity[2])) <= 0.6 * dt + 2e-5
        assert abs(current.yawspeed - previous.yawspeed) <= 1.0 * dt + 2e-5
        assert speed(current) <= max_speed + 1e-5
        assert abs(current.velocity[2]) <= 1.0 + 1e-5
        assert abs(current.yawspeed) <= 0.8 + 1e-5
        assert all(math.isnan(value) for value in current.acceleration)


@pytest.mark.parametrize('fallback_png', [False, True])
def test_commands_remain_bounded_through_turn_loss_recovery_and_done(tmp_path, fallback_png):
    with Controller(tmp_path, tmp_path / 'unused.csv', fallback_png=fallback_png) as controller:
        controller.follow_commands = True  # ideal velocity response, no Gazebo/flight hardware
        controller.wait_for('进入 INTERCEPT')
        controller.spin_for(2.0)
        controller.box = (1440, 220, 80, 80)
        controller.spin_for(0.7)
        assert speed(controller.commands[-1][1]) > 0.4
        controller.detection = 'invalid'
        loss_start = time.monotonic()
        controller.spin_for(0.35)
        coasting = [msg for stamp, msg in controller.commands if loss_start + 0.1 < stamp < loss_start + 0.3]
        assert len(coasting) >= 5
        assert max(speed(msg) for msg in coasting) - min(speed(msg) for msg in coasting) < 1e-5
        before_brake = speed(controller.commands[-1][1])
        controller.spin_for(0.8)
        assert speed(controller.commands[-1][1]) < before_brake - 0.1
        controller.wait_for('[TRACK_LOST]')
        controller.detection = 'valid'
        controller.box = (400, 700, 60, 60)
        controller.wait_for('恢复 INTERCEPT')
        controller.spin_for(0.7)
        controller.approach_and_check_success()
        controller.spin_for(0.3)
    assert len(controller.commands) > 200
    assert_command_limits(controller.commands)


def test_final_speed_limit_can_be_lower_than_policy_decode_minimum(tmp_path):
    with Controller(tmp_path, tmp_path / 'unused.csv', parameters={'max_speed': 1.2}) as controller:
        controller.follow_commands = True
        controller.wait_for('进入 INTERCEPT')
        controller.spin_for(2.5)
    assert_command_limits(controller.commands, max_speed=1.2)
    assert max(speed(message) for _, message in controller.commands) > 1.0
    assert 'cmd_source=policy' in controller.output
