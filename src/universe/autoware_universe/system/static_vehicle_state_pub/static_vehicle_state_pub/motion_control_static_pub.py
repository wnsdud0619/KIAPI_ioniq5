#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from autoware_adapi_v1_msgs.msg import MotionState
from autoware_vehicle_msgs.msg import ControlModeReport


class MotionControlStaticPublisher(Node):
    def __init__(self):
        super().__init__('static_motion_control_publisher')

        # parameters
        self.declare_parameter('publish_motion_state', True)
        self.declare_parameter('publish_control_mode', True)
        self.declare_parameter('publish_rate', 1.0)

        self.publish_motion_state = self.get_parameter(
            'publish_motion_state').get_parameter_value().bool_value
        self.publish_control_mode = self.get_parameter(
            'publish_control_mode').get_parameter_value().bool_value
        self.publish_rate = self.get_parameter(
            'publish_rate').get_parameter_value().double_value

        # publishers
        if self.publish_motion_state:
            self.motion_state_pub = self.create_publisher(
                MotionState,
                '/api/motion/state',
                10
            )
            self.get_logger().info('MotionState publisher ENABLED')

        if self.publish_control_mode:
            self.control_mode_pub = self.create_publisher(
                ControlModeReport,
                '/vehicle/status/control_mode',
                10
            )
            self.get_logger().info('ControlModeReport publisher ENABLED')

        # timer
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.timer_callback
        )

    def timer_callback(self):
        now = self.get_clock().now().to_msg()

        # 1) MotionState
        if self.publish_motion_state:
            msg = MotionState()
            msg.stamp = now
            msg.state = 3   # bash 명령과 동일

            self.motion_state_pub.publish(msg)

        # 2) ControlModeReport
        if self.publish_control_mode:
            msg = ControlModeReport()
            msg.stamp = now
            msg.mode = 1   # bash 명령과 동일

            self.control_mode_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotionControlStaticPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

