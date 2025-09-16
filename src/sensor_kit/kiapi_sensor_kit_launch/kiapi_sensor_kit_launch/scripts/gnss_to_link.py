#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class GPStoTF(Node):

    def __init__(self):
        super().__init__('kiapi_gps_to_link')

        self._pose_topic = '/sensing/gnss/pose_with_covariance'
        self._imu_topic = '/novatel/oem7/imu/data'

        self.tf_broadcaster = TransformBroadcaster(self)

        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT)

        # store latest IMU orientation
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0

        # subscriptions
        self._sub_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            self._pose_topic,
            self.PoseCovStmpCallback,
            qos_profile=qos_profile)

        self._sub_imu = self.create_subscription(
            Imu,
            self._imu_topic,
            self.ImuCallback,
            qos_profile=qos_profile)

        self.get_logger().info("GPStoTF node initialized")

    def ImuCallback(self, msg: Imu):
        self.qx = msg.orientation.x
        self.qy = msg.orientation.y
        self.qz = msg.orientation.z
        self.qw = msg.orientation.w

    def PoseCovStmpCallback(self, msg: PoseWithCovarianceStamped):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # use latest IMU orientation
        t.transform.rotation.x = self.qx
        t.transform.rotation.y = self.qy
        t.transform.rotation.z = self.qz
        t.transform.rotation.w = self.qw

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().debug("Broadcasted transform map->base_link")


def main(args=None):
    rclpy.init(args=args)
    node = GPStoTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
