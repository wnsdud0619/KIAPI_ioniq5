#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from autoware_perception_msgs.msg import (
    TrafficLightGroupArray,
    TrafficLightGroup,
    TrafficLightElement
)


class StaticTrafficLightPublisher(Node):
    def __init__(self):
        super().__init__('static_tl_publisher')  # ← 노드명

        self.publisher_ = self.create_publisher(
            TrafficLightGroupArray,
            '/perception/traffic_light_recognition/traffic_signals',
            10
        )

        self.timer = self.create_timer(1.0, self.timer_callback)

        self.group_ids = [
            1270012619,
            1270012642,
            1270012633,
            1270012683,
            1270012702,
            1270012723,
            1270012745,
            1270012766,
            1270012787,
            1270012809]

        self.get_logger().info('Static Traffic Light Publisher started (1 Hz)')

    def timer_callback(self):
        msg = TrafficLightGroupArray()

        # ROS Time (simulation / system time 모두 대응)
        msg.stamp = self.get_clock().now().to_msg()

        for gid in self.group_ids:
            group = TrafficLightGroup()
            group.traffic_light_group_id = gid

            element = TrafficLightElement()
            element.color = 3      # 1=red, 2=amber, 3=green
            element.shape = 1
            element.status = 2
            element.confidence = 1.0

            group.elements.append(element)
            msg.traffic_light_groups.append(group)

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StaticTrafficLightPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

