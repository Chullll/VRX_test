#!/usr/bin/env python3
"""Simple thruster control node with optional USV data collection."""

from importlib import import_module

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


DATA_COLLECTOR_MODULE = 'wamv_control.data_collector'
DATA_COLLECTOR_CLASS = 'USVDataCollector'


class SimpleThrusterControl(Node):
    """Publish simple thrust commands with optional data collection."""

    def __init__(self, enable_data_collection=False):
        """Create publishers, the optional collector, and the control timer."""
        super().__init__('simple_thruster_control')

        self.left_thrust_pub = self.create_publisher(
            Float64,
            '/wamv/thrusters/left/thrust',
            10
        )

        self.right_thrust_pub = self.create_publisher(
            Float64,
            '/wamv/thrusters/right/thrust',
            10
        )

        self.left_pos_pub = self.create_publisher(
            Float64,
            '/wamv/thrusters/left/pos',
            10
        )

        self.right_pos_pub = self.create_publisher(
            Float64,
            '/wamv/thrusters/right/pos',
            10
        )

        self.data_collector = None

        # 如果终端选择采集数据，则在控制计时开始前启动采集接口。
        # 采集脚本建议提供 USVDataCollector(node) 类，并可选实现
        # start()/stop()，在其中订阅全局位姿、IMU 等 topic 并写入文件。
        if enable_data_collection:
            self.start_data_collection()

        # 使用 use_sim_time 时，节点刚创建时 ROS clock 可能仍为 0。
        # 如果此时记录 start_time，仿真已经运行较久时会导致 t 直接大于
        # 30 s，控制逻辑立即进入停止分支。因此控制起点放到第一次
        # control_loop 中，用收到的当前时钟初始化。
        self.start_time = None
        self.timer = self.create_timer(0.1, self.control_loop)

    def start_data_collection(self):
        """Import and start the reserved data collection interface."""
        try:
            collector_module = import_module(DATA_COLLECTOR_MODULE)
            collector_class = getattr(collector_module, DATA_COLLECTOR_CLASS)
            self.data_collector = collector_class(self)

            # 若采集器需要显式打开文件、写表头或启动缓存线程，可在这里完成。
            if hasattr(self.data_collector, 'start'):
                self.data_collector.start()

        except Exception as exc:
            self.get_logger().error(
                'Failed to load data collector interface: '
                f'{DATA_COLLECTOR_MODULE}.{DATA_COLLECTOR_CLASS}. '
                f'Please check the data collection script. Error: {exc}'
            )
            raise

        self.get_logger().info(
            'Data collection enabled. Expected data: global position, '
            'global orientation quaternion, Euler angles, IMU angular '
            'velocity, and IMU linear acceleration.'
        )

    def stop_data_collection(self):
        """Stop the data collector if it provides a stop hook."""
        if self.data_collector is None:
            return

        # 退出控制脚本前关闭文件句柄/刷新缓存，避免采集数据丢失。
        if hasattr(self.data_collector, 'stop'):
            self.data_collector.stop()

    def publish_command(self, left_thrust, right_thrust, left_pos, right_pos):
        """Publish thrust and azimuth position commands to both thrusters."""
        msg = Float64()

        msg.data = left_thrust
        self.left_thrust_pub.publish(msg)

        msg.data = right_thrust
        self.right_thrust_pub.publish(msg)

        msg.data = left_pos
        self.left_pos_pub.publish(msg)

        msg.data = right_pos
        self.right_pos_pub.publish(msg)

    def control_loop(self):
        """Run the time-based open-loop thruster command sequence."""
        now = self.get_clock().now()

        if self.start_time is None:
            self.start_time = now
            self.get_logger().info(
                'Control timer started from current node clock.'
            )

        t = (now - self.start_time).nanoseconds * 1e-9

        # # 0~10 s：直行
        # if t < 10.0:
        #     left_thrust = 40.0
        #     right_thrust = 40.0
        #     left_pos = 0.0
        #     right_pos = 0.0

        # # 10~20 s：右转
        # elif t < 20.0:
        #     left_thrust = 40.0
        #     right_thrust = 10.0
        #     left_pos = 0.0
        #     right_pos = 0.0

        # # 20~30 s：左转
        # elif t < 30.0:
        #     left_thrust = 10.0
        #     right_thrust = 40.0
        #     left_pos = 0.0
        #     right_pos = 0.0

        # # 30 s 以后：停止主动推进
        # else:
        #     left_thrust = 0.0
        #     right_thrust = 0.0
        #     left_pos = 0.0
        #     right_pos = 0.0

        if t < 30.0:
            left_thrust = 40.0
            right_thrust = 40.0
            left_pos = 0.0
            right_pos = 0.0
        elif t < 60.0:
            left_thrust = 40.0
            right_thrust = 35.0
            left_pos = 0.0
            right_pos = 0.0
        else:
            left_thrust = 0.0
            right_thrust = 0.0
            left_pos = 0.0
            right_pos = 0.0



        self.publish_command(left_thrust, right_thrust, left_pos, right_pos)

        self.get_logger().info(
            f't={t:.1f}s, '
            f'left_thrust={left_thrust:.2f}, '
            f'right_thrust={right_thrust:.2f}, '
            f'left_pos={left_pos:.2f}, '
            f'right_pos={right_pos:.2f}'
        )


def ask_enable_data_collection():
    """Ask whether data collection should be enabled before control starts."""
    try:
        prompt = '是否在控制开始前启用 USV 数据采集？[y/N]: '
        answer = input(prompt)
    except EOFError:
        return False

    return answer.strip().lower() in ('y', 'yes', '是', '是的')


def main(args=None):
    """Start the simple thruster control node."""
    # 先做终端询问，再初始化 ROS 2。这样即使 ROS 参数较多，也能明确看到
    # 是否启用数据采集的交互提示；真正的节点参数仍由 rclpy.init 解析。
    enable_data_collection = ask_enable_data_collection()
    rclpy.init(args=args)
    node = SimpleThrusterControl(enable_data_collection)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publish_command(0.0, 0.0, 0.0, 0.0)
        node.get_logger().info('Stopped. Published zero thrust command.')
    finally:
        node.stop_data_collection()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
