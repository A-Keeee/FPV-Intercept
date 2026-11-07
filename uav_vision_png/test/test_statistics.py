"""Exercise the real node in an isolated ROS domain, with synthetic PX4 inputs."""

import math
import os
import signal
import subprocess
import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import HoverThrustEstimate, SensorGps, VehicleLocalPosition, VehicleOdometry
from uav_common_msg.msg import RectMsg


class Controller:
    def __init__(self, directory, csv_path, record_csv=False):
        self.directory = directory
        self.csv_path = csv_path
        self.record_csv = record_csv
        self.close_target = False
        self.detection = 'valid'

    def __enter__(self):
        # Isolate all arm/offboard publications from the user's PX4 session.
        os.environ['ROS_DOMAIN_ID'] = str(140 + os.getpid() % 50)
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        rclpy.init()
        self.node = Node('statistics_regression')
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
        self.process = subprocess.Popen([
            os.environ['VPNG_NODE_EXECUTABLE'], '--ros-args',
            '-p', f'csv_path:={self.csv_path}',
            '-p', f'record_csv:={str(self.record_csv).lower()}',
        ], stdout=self.output_file, stderr=subprocess.STDOUT, env=environment)
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
            detection.x = 940
            detection.y = 520
            detection.width = detection.height = 40
        else:
            detection.x = detection.y = detection.width = detection.height = -1
        self.publishers['detection'].publish(detection)

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
        controller.approach_and_check_success()
    assert csv_path.read_text() == 'previous session must remain unchanged\n'
    assert 'csv=disabled' in controller.output
    assert 'event=summary hit=true' in controller.output
    log_files = list((tmp_path / 'ros_logs').glob('uav_vision_png_*.log'))
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
    assert len(rows) <= 15  # the sink is sampled at 10 Hz, not the 200 Hz control rate
    assert '# 命中时刻(s),' in csv_path.read_text()
