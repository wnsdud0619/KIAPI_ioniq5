#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, Quaternion
import math
import time

class RoughGoalPublisher(Node):
    def __init__(self):
        super().__init__('rough_goal_node')

        # Parameters
        self.waypoint_topic = '/platoon_waypoints_map'
        self.goal_topic = '/rviz/routing/rough_goal'
        self.min_distance = 5.0
        self.publish_interval = 1.0

        # State
        self.last_published_pose = None
        self.last_publish_time = 0.0

        # Subscriber & Publisher
        self.sub_waypoints = self.create_subscription(
            PoseArray,
            self.waypoint_topic,
            self.waypoints_callback,
            10
        )
        self.pub_goal = self.create_publisher(
            PoseStamped,
            self.goal_topic,
            10
        )

        self.get_logger().info('Rough Goal Publisher Initialized')

    def waypoints_callback(self, msg: PoseArray):
        if not msg.poses:
            self.get_logger().warn('Received empty waypoint array, skipping.')
            return

        last_pose = msg.poses[-1]

        # 거리 조건 확인
        if self.last_published_pose is not None:
            dx = last_pose.position.x - self.last_published_pose.position.x
            dy = last_pose.position.y - self.last_published_pose.position.y
            dist = math.hypot(dx, dy)
            if dist < self.min_distance:
                return

        # 시간 조건 확인
        now = time.time()
        if now - self.last_publish_time < self.publish_interval:
            return

        # ROS yaw 계산
        ros_yaw_deg = self.get_yaw_deg(last_pose)

        # ENU yaw 보정
        enu_yaw_deg = (90 - ros_yaw_deg) % 360
        enu_yaw_rad = math.radians(enu_yaw_deg)

        # 보정된 yaw로 quaternion 재생성
        last_pose.orientation = self.yaw_to_quaternion(enu_yaw_rad)

        # Publish
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = msg.header.frame_id
        goal_msg.pose = last_pose

        self.pub_goal.publish(goal_msg)
        self.last_published_pose = last_pose
        self.last_publish_time = now

        self.get_logger().info(
            f'Published rough goal at x={last_pose.position.x:.2f}, '
            f'y={last_pose.position.y:.2f}, yaw={enu_yaw_deg:.2f}° (ENU corrected)'
        )

    @staticmethod
    def get_yaw_deg(pose) -> float:
        """Pose.orientation에서 ROS yaw(degree) 계산"""
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(yaw_rad)

    @staticmethod
    def yaw_to_quaternion(yaw_rad: float) -> Quaternion:
        """Yaw(rad) -> Quaternion (roll=pitch=0)"""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw_rad / 2.0)
        q.w = math.cos(yaw_rad / 2.0)
        return q

def main(args=None):
    rclpy.init(args=args)
    node = RoughGoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

