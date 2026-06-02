#!/usr/bin/env python3
"""Collect WAM-V simulation pose and IMU data into a CSV file."""

import csv
from dataclasses import dataclass
from datetime import datetime
import math
from pathlib import Path
import shutil
import subprocess
import threading

from sensor_msgs.msg import Imu


DEFAULT_GAZEBO_POSE_TOPIC = '/world/sydney_regatta/dynamic_pose/info'
DEFAULT_IMU_TOPIC = '/wamv/sensors/imu/imu/data'
DEFAULT_TARGET_MODEL_NAME = 'wamv'
DEFAULT_OUTPUT_DIR = Path.home() / 'vrx_ws' / 'data'
DEFAULT_SAMPLE_PERIOD = 0.1
DEFAULT_FLUSH_EVERY_N = 10

CSV_HEADER = [
    'sim_time_sec',
    'elapsed_time_sec',
    'global_x',
    'global_y',
    'global_z',
    'global_qx',
    'global_qy',
    'global_qz',
    'global_qw',
    'roll',
    'pitch',
    'yaw',
    'global_vx',
    'global_vy',
    'global_vz',
    'imu_angular_velocity_x',
    'imu_angular_velocity_y',
    'imu_angular_velocity_z',
    'imu_linear_acceleration_x',
    'imu_linear_acceleration_y',
    'imu_linear_acceleration_z',
    'imu_angular_acceleration_x',
    'imu_angular_acceleration_y',
    'imu_angular_acceleration_z',
]


@dataclass
class GazeboPoseSample:
    """Raw global pose parsed from Gazebo."""

    stamp: float
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass
class ImuSample:
    """Raw IMU sample parsed from sensor_msgs/msg/Imu."""

    stamp: float
    wx: float
    wy: float
    wz: float
    ax: float
    ay: float
    az: float


@dataclass
class PoseEstimate:
    """Pose with Euler angles and global linear velocity."""

    stamp: float
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    roll: float
    pitch: float
    yaw: float
    vx: float
    vy: float
    vz: float


@dataclass
class EstimatedState:
    """Complete state snapshot written to CSV."""

    sim_time: float
    elapsed_time: float
    pose: PoseEstimate
    imu: ImuSample
    angular_acceleration: tuple


class GazeboPoseReader:
    """Read WAM-V global pose from Gazebo Transport text output."""

    def __init__(self, topic, target_model_name, time_source, on_pose, logger):
        """Create a reader for `gz topic -e -t <topic>`."""
        self.topic = topic
        self.target_model_name = target_model_name
        self.time_source = time_source
        self.on_pose = on_pose
        self.logger = logger

        self.running = False
        self.process = None
        self.stdout_thread = None
        self.stderr_thread = None

    def start(self):
        """Start the Gazebo topic reader subprocess."""
        if self.running:
            return

        gazebo_command = self._find_gazebo_command()
        command = [
            gazebo_command,
            'topic',
            '-e',
            '-t',
            self.topic,
        ]

        try:
            self.process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1
            )
        except OSError as exc:
            raise RuntimeError(
                f'Failed to start Gazebo pose reader: {command}'
            ) from exc

        self.running = True
        self.stdout_thread = threading.Thread(
            target=self._read_stdout,
            daemon=True
        )
        self.stderr_thread = threading.Thread(
            target=self._read_stderr,
            daemon=True
        )
        self.stdout_thread.start()
        self.stderr_thread.start()

    def stop(self):
        """Stop the Gazebo topic reader subprocess."""
        self.running = False
        if self.process is None:
            return

        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2.0)

        for thread in (self.stdout_thread, self.stderr_thread):
            if thread is not None:
                thread.join(timeout=1.0)

        self.process = None
        self.stdout_thread = None
        self.stderr_thread = None

    def _find_gazebo_command(self):
        # 新版 Gazebo 使用 gz；旧版 Ignition 可能仍使用 ign。
        for command in ('gz', 'ign'):
            if shutil.which(command):
                return command

        raise RuntimeError('Cannot find gz or ign command in PATH.')

    def _read_stderr(self):
        if self.process is None or self.process.stderr is None:
            return

        for raw_line in self.process.stderr:
            if not self.running:
                break

            line = raw_line.strip()
            if line:
                self.logger.warn(f'Gazebo pose reader: {line}')

    def _read_stdout(self):
        if self.process is None or self.process.stdout is None:
            return

        current_pose = None
        current_section = None

        try:
            for raw_line in self.process.stdout:
                if not self.running:
                    break

                line = raw_line.strip()
                if line == 'pose {':
                    current_pose = self._new_pose_candidate()
                    current_section = None
                    continue

                if current_pose is None:
                    continue

                if line == 'position {':
                    current_section = 'position'
                    continue

                if line == 'orientation {':
                    current_section = 'orientation'
                    continue

                if line == '}':
                    current_section, current_pose = self._handle_close_brace(
                        current_section,
                        current_pose
                    )
                    continue

                self._parse_pose_line(current_pose, current_section, line)

        except Exception as exc:
            if self.running:
                self.logger.error(f'Gazebo pose stream parser stopped: {exc}')

    def _handle_close_brace(self, current_section, current_pose):
        if current_section is not None:
            return None, current_pose

        self._publish_pose_if_target(current_pose)
        return None, None

    def _new_pose_candidate(self):
        return {
            'name': '',
            'position': {'x': None, 'y': None, 'z': None},
            'orientation': {'x': None, 'y': None, 'z': None, 'w': None},
        }

    def _parse_pose_line(self, current_pose, current_section, line):
        if line.startswith('name:'):
            current_pose['name'] = self._parse_name(line)
            return

        if current_section not in ('position', 'orientation'):
            return

        key, value = self._parse_float_field(line)
        if key in current_pose[current_section]:
            current_pose[current_section][key] = value

    def _parse_name(self, line):
        _, value = line.split(':', 1)
        return value.strip().strip('"')

    def _parse_float_field(self, line):
        if ':' not in line:
            return None, None

        key, value = line.split(':', 1)
        try:
            return key.strip(), float(value.strip())
        except ValueError:
            return None, None

    def _publish_pose_if_target(self, current_pose):
        if current_pose['name'] != self.target_model_name:
            return

        position = current_pose['position']
        orientation = current_pose['orientation']
        if not self._has_complete_pose(position, orientation):
            return

        # Gazebo 话题文本本身没有 stamp，这里使用 ROS clock 当前值。
        # 当节点启用 use_sim_time 时，该值就是仿真时间。
        sample = GazeboPoseSample(
            stamp=self.time_source(),
            x=position['x'],
            y=position['y'],
            z=position['z'],
            qx=orientation['x'],
            qy=orientation['y'],
            qz=orientation['z'],
            qw=orientation['w']
        )
        self.on_pose(sample)

    def _has_complete_pose(self, position, orientation):
        values = (
            position['x'],
            position['y'],
            position['z'],
            orientation['x'],
            orientation['y'],
            orientation['z'],
            orientation['w'],
        )
        return all(value is not None for value in values)


class ImuReader:
    """Subscribe to the ROS 2 IMU topic and expose raw IMU samples."""

    def __init__(self, node, topic, time_source, on_imu):
        """Create the IMU reader."""
        self.node = node
        self.topic = topic
        self.time_source = time_source
        self.on_imu = on_imu
        self.subscription = None

    def start(self):
        """Start the IMU subscription."""
        if self.subscription is not None:
            return

        # IMU 是标准 ROS 2 topic，直接订阅 sensor_msgs/msg/Imu。
        self.subscription = self.node.create_subscription(
            Imu,
            self.topic,
            self._imu_callback,
            10
        )

    def stop(self):
        """Stop the IMU subscription."""
        if self.subscription is None:
            return

        self.node.destroy_subscription(self.subscription)
        self.subscription = None

    def _imu_callback(self, msg):
        stamp = self._stamp_to_seconds(msg.header.stamp)
        if stamp <= 0.0:
            stamp = self.time_source()

        sample = ImuSample(
            stamp=stamp,
            wx=msg.angular_velocity.x,
            wy=msg.angular_velocity.y,
            wz=msg.angular_velocity.z,
            ax=msg.linear_acceleration.x,
            ay=msg.linear_acceleration.y,
            az=msg.linear_acceleration.z
        )
        self.on_imu(sample)

    def _stamp_to_seconds(self, stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class StateEstimator:
    """Compute Euler angles, global velocity, and angular acceleration."""

    def __init__(self):
        """Initialize estimator state."""
        self.lock = threading.Lock()
        self.latest_pose = None
        self.latest_imu = None
        self.latest_angular_acceleration = (0.0, 0.0, 0.0)

    def update_pose(self, raw_pose):
        """Update pose estimate from one Gazebo global pose sample."""
        qx, qy, qz, qw = self._normalize_quaternion(
            raw_pose.qx,
            raw_pose.qy,
            raw_pose.qz,
            raw_pose.qw
        )
        roll, pitch, yaw = self._quaternion_to_euler(qx, qy, qz, qw)

        with self.lock:
            # 全局线速度由 Gazebo 全局位置差分得到。
            vx, vy, vz = self._calculate_global_velocity(
                self.latest_pose,
                raw_pose
            )
            self.latest_pose = PoseEstimate(
                stamp=raw_pose.stamp,
                x=raw_pose.x,
                y=raw_pose.y,
                z=raw_pose.z,
                qx=qx,
                qy=qy,
                qz=qz,
                qw=qw,
                roll=roll,
                pitch=pitch,
                yaw=yaw,
                vx=vx,
                vy=vy,
                vz=vz
            )

    def update_imu(self, sample):
        """Update IMU estimate and angular acceleration."""
        with self.lock:
            # 角加速度由 IMU 角速度差分得到，单位近似为 rad/s^2。
            self.latest_angular_acceleration = (
                self._calculate_angular_acceleration(
                    self.latest_imu,
                    sample
                )
            )
            self.latest_imu = sample

    def snapshot(self, sim_time, start_time):
        """Return the latest complete estimated state."""
        with self.lock:
            if self.latest_pose is None or self.latest_imu is None:
                return None

            return EstimatedState(
                sim_time=sim_time,
                elapsed_time=sim_time - start_time,
                pose=self.latest_pose,
                imu=self.latest_imu,
                angular_acceleration=self.latest_angular_acceleration
            )

    def has_pose(self):
        """Return whether at least one global pose has arrived."""
        with self.lock:
            return self.latest_pose is not None

    def has_imu(self):
        """Return whether at least one IMU sample has arrived."""
        with self.lock:
            return self.latest_imu is not None

    def _normalize_quaternion(self, qx, qy, qz, qw):
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 0.0:
            return 0.0, 0.0, 0.0, 1.0

        return qx / norm, qy / norm, qz / norm, qw / norm

    def _quaternion_to_euler(self, qx, qy, qz, qw):
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def _calculate_global_velocity(self, old_pose, new_pose):
        if old_pose is None:
            return 0.0, 0.0, 0.0

        dt = new_pose.stamp - old_pose.stamp
        if dt <= 1e-6:
            return old_pose.vx, old_pose.vy, old_pose.vz

        return (
            (new_pose.x - old_pose.x) / dt,
            (new_pose.y - old_pose.y) / dt,
            (new_pose.z - old_pose.z) / dt,
        )

    def _calculate_angular_acceleration(self, old_imu, new_imu):
        if old_imu is None:
            return 0.0, 0.0, 0.0

        dt = new_imu.stamp - old_imu.stamp
        if dt <= 1e-6:
            return self.latest_angular_acceleration

        return (
            (new_imu.wx - old_imu.wx) / dt,
            (new_imu.wy - old_imu.wy) / dt,
            (new_imu.wz - old_imu.wz) / dt,
        )


class CsvLogger:
    """Write estimated state snapshots to CSV."""

    def __init__(self, output_dir, flush_every_n):
        """Create a CSV logger."""
        self.output_dir = Path(output_dir).expanduser()
        self.flush_every_n = max(1, int(flush_every_n))
        self.output_path = None
        self.csv_file = None
        self.csv_writer = None
        self.rows_since_flush = 0

    def start(self):
        """Open the CSV file and write the header."""
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.output_path = self._make_output_path()
        self.csv_file = open(
            self.output_path,
            'w',
            newline='',
            encoding='utf-8'
        )
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(CSV_HEADER)

    def stop(self):
        """Flush and close the CSV file."""
        if self.csv_file is None:
            return

        self.csv_file.flush()
        self.csv_file.close()
        self.csv_file = None
        self.csv_writer = None

    def write_state(self, state):
        """Write one estimated state row."""
        if self.csv_file is None or self.csv_writer is None:
            return

        self.csv_writer.writerow(self._state_to_row(state))
        self.rows_since_flush += 1

        # 不再每行 flush，降低磁盘写入开销；退出时仍会强制 flush。
        if self.rows_since_flush >= self.flush_every_n:
            self.csv_file.flush()
            self.rows_since_flush = 0

    def _make_output_path(self):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        return self.output_dir / f'usv_data_{timestamp}.csv'

    def _state_to_row(self, state):
        pose = state.pose
        imu = state.imu
        angular_acceleration = state.angular_acceleration
        values = [
            state.sim_time,
            state.elapsed_time,
            pose.x,
            pose.y,
            pose.z,
            pose.qx,
            pose.qy,
            pose.qz,
            pose.qw,
            pose.roll,
            pose.pitch,
            pose.yaw,
            pose.vx,
            pose.vy,
            pose.vz,
            imu.wx,
            imu.wy,
            imu.wz,
            imu.ax,
            imu.ay,
            imu.az,
            angular_acceleration[0],
            angular_acceleration[1],
            angular_acceleration[2],
        ]
        return [self._format_float(value) for value in values]

    def _format_float(self, value):
        return f'{value:.9f}'


class USVDataCollector:
    """Organize readers, estimator, and logger for WAM-V data collection."""

    def __init__(self, node):
        """Initialize the data collection pipeline."""
        self.node = node

        self.gazebo_pose_topic = self._declare_string_parameter(
            'data_gazebo_pose_topic',
            DEFAULT_GAZEBO_POSE_TOPIC
        )
        self.imu_topic = self._declare_string_parameter(
            'data_imu_topic',
            DEFAULT_IMU_TOPIC
        )
        self.target_model_name = self._declare_string_parameter(
            'data_target_model_name',
            DEFAULT_TARGET_MODEL_NAME
        )
        output_dir = self._declare_string_parameter(
            'data_output_dir',
            str(DEFAULT_OUTPUT_DIR)
        )
        self.sample_period = self._declare_float_parameter(
            'data_sample_period',
            DEFAULT_SAMPLE_PERIOD
        )
        flush_every_n = self._declare_int_parameter(
            'data_flush_every_n',
            DEFAULT_FLUSH_EVERY_N
        )

        self.running = False
        self.start_stamp = None
        self.write_timer = None
        self.warned_missing_pose = False
        self.warned_missing_imu = False

        self.estimator = StateEstimator()
        self.csv_logger = CsvLogger(output_dir, flush_every_n)
        self.pose_reader = GazeboPoseReader(
            self.gazebo_pose_topic,
            self.target_model_name,
            self._ros_time_seconds,
            self.estimator.update_pose,
            self.node.get_logger()
        )
        self.imu_reader = ImuReader(
            self.node,
            self.imu_topic,
            self._ros_time_seconds,
            self.estimator.update_imu
        )

    def start(self):
        """Start readers, estimator timer, and CSV logger."""
        if self.running:
            return

        try:
            self._warn_if_not_using_sim_time()
            self.running = True
            self.start_stamp = self._ros_time_seconds()

            self.csv_logger.start()
            self.imu_reader.start()
            self.pose_reader.start()

            # 用固定频率写 CSV，避免 IMU 高频回调直接触发大量磁盘写入。
            self.write_timer = self.node.create_timer(
                self.sample_period,
                self._write_sample
            )
        except Exception:
            self.stop()
            raise

        self.node.get_logger().info(
            f'USV data collector started. CSV: {self.csv_logger.output_path}'
        )

    def stop(self):
        """Stop all data collection components."""
        self.running = False

        if self.write_timer is not None:
            self.node.destroy_timer(self.write_timer)
            self.write_timer = None

        self.imu_reader.stop()
        self.pose_reader.stop()
        self.csv_logger.stop()

        if self.csv_logger.output_path is not None:
            self.node.get_logger().info(
                'USV data collector stopped. '
                f'CSV saved to {self.csv_logger.output_path}'
            )

    def _write_sample(self):
        sim_time = self._ros_time_seconds()
        state = self.estimator.snapshot(sim_time, self.start_stamp)
        if state is None:
            self._warn_if_data_missing()
            return

        self.csv_logger.write_state(state)

    def _ros_time_seconds(self):
        # 当节点设置 use_sim_time:=true 且 /clock 可用时，这里返回仿真时间。
        now = self.node.get_clock().now()
        return float(now.nanoseconds) * 1e-9

    def _warn_if_data_missing(self):
        if not self.estimator.has_pose() and not self.warned_missing_pose:
            self.node.get_logger().warn(
                'Waiting for Gazebo global pose data of target "wamv".'
            )
            self.warned_missing_pose = True

        if not self.estimator.has_imu() and not self.warned_missing_imu:
            self.node.get_logger().warn(
                f'Waiting for IMU data on {self.imu_topic}.'
            )
            self.warned_missing_imu = True

    def _warn_if_not_using_sim_time(self):
        if not self.node.has_parameter('use_sim_time'):
            self.node.get_logger().warn(
                'use_sim_time is not set. Run with '
                '--ros-args -p use_sim_time:=true to record simulation time.'
            )
            return

        use_sim_time = self.node.get_parameter('use_sim_time').value
        if not use_sim_time:
            self.node.get_logger().warn(
                'use_sim_time is false. CSV timestamps will use the node clock '
                'instead of Gazebo simulation time.'
            )

    def _declare_string_parameter(self, name, default_value):
        try:
            return self.node.declare_parameter(name, default_value).value
        except Exception:
            return self.node.get_parameter(name).value

    def _declare_float_parameter(self, name, default_value):
        try:
            parameter = self.node.declare_parameter(name, default_value)
            return float(parameter.value)
        except Exception:
            return float(self.node.get_parameter(name).value)

    def _declare_int_parameter(self, name, default_value):
        try:
            parameter = self.node.declare_parameter(name, default_value)
            return int(parameter.value)
        except Exception:
            return int(self.node.get_parameter(name).value)
