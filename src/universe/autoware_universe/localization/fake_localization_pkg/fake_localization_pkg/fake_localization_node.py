#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import (
    PoseStamped, Point, Quaternion,
    TwistStamped, Vector3, AccelWithCovarianceStamped
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf2_ros
from tf2_ros import TransformException


class FakeLocalizationNode(Node):
    def __init__(self):
        super().__init__('fake_localization_node')

        # QoS 설정
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        # Publishers
        self.pub_state = self.create_publisher(LocalizationInitializationState, '/localization/initialization_state', qos)
        self.pub_pose = self.create_publisher(PoseStamped, '/localization/pose_twist_fusion_filter/pose', qos)
        self.pub_twist = self.create_publisher(TwistStamped, '/localization/pose_twist_fusion_filter/twist', qos)
        self.pub_vehicle_twist = self.create_publisher(TwistStamped, '/vehicle/status/twist', qos)   # ✅ 추가
        self.pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.pub_kinematic_state = self.create_publisher(Odometry, '/localization/kinematic_state', qos)
        self.pub_accel = self.create_publisher(AccelWithCovarianceStamped, '/localization/acceleration', qos)

        # Subscriber: NovAtel odometry (twist만 사용)
        self.sub_odom = self.create_subscription(Odometry, '/novatel/oem7/odom', self.odom_callback, 10)

        # Subscriber: NovAtel IMU
        self.sub_imu = self.create_subscription(Imu, '/novatel/oem7/imu/data', self.imu_callback, 10)

        # TF Buffer + Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 최근 IMU 데이터 저장
        self.latest_imu = None

    def imu_callback(self, msg: Imu):
        """IMU 데이터 수신 시 저장"""
        self.latest_imu = msg

    def odom_callback(self, msg: Odometry):
        now = self.get_clock().now().to_msg()

        # --- Localization InitializationState ---
        msg_state = LocalizationInitializationState()
        msg_state.stamp = now
        msg_state.state = LocalizationInitializationState.INITIALIZED
        self.pub_state.publish(msg_state)

        # --- TF에서 map->base_link pose 조회 ---
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform map->base_link: {ex}')
            return

        # --- PoseStamped ---
        msg_pose = PoseStamped()
        msg_pose.header.stamp = now
        msg_pose.header.frame_id = "map"
        msg_pose.pose.position = Point(x=x, y=y, z=z)
        msg_pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        self.pub_pose.publish(msg_pose)

        # --- TwistStamped (odometry twist 그대로) ---
        msg_twist = TwistStamped()
        msg_twist.header.stamp = now
        msg_twist.header.frame_id = "base_link"
        msg_twist.twist = msg.twist.twist
        self.pub_twist.publish(msg_twist)
        #self.pub_vehicle_twist.publish(msg_twist)   # ✅ /vehicle/status/twist 추가 퍼블리시

        # --- KinematicState (pose: TF, twist: odom) ---
        msg_kin = Odometry()
        msg_kin.header.stamp = now
        msg_kin.header.frame_id = "map"
        msg_kin.child_frame_id = "base_link"
        msg_kin.pose.pose.position = Point(x=x, y=y, z=z)
        msg_kin.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        msg_kin.twist.twist = msg.twist.twist
        self.pub_kinematic_state.publish(msg_kin)

        # --- Acceleration (IMU 기반) ---
        accel_msg = AccelWithCovarianceStamped()
        accel_msg.header.stamp = now
        accel_msg.header.frame_id = "map"

        if self.latest_imu is not None:
            accel_msg.accel.accel.linear = self.latest_imu.linear_acceleration
            accel_msg.accel.accel.angular = self.latest_imu.angular_velocity

            cov = [0.0] * 36
            for i in range(3):
                for j in range(3):
                    cov[i*6 + j] = self.latest_imu.linear_acceleration_covariance[i*3 + j]
            for i in range(3):
                for j in range(3):
                    cov[3*6 + i*6 + j + 3] = self.latest_imu.angular_velocity_covariance[i*3 + j]

            accel_msg.accel.covariance = cov
        else:
            accel_msg.accel.accel.linear = Vector3()
            accel_msg.accel.accel.angular = Vector3()
            accel_msg.accel.covariance = [0.0] * 36

        self.pub_accel.publish(accel_msg)

        # --- Diagnostics ---
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = now
        status_pose = DiagnosticStatus()
        status_pose.level = DiagnosticStatus.OK
        status_pose.name = 'topic_state_monitor_pose_twist_fusion_filter_pose: localization_topic_status'
        status_pose.hardware_id = 'topic_state_monitor'
        status_pose.message = 'OK'
        status_pose.values = [
            KeyValue(key='topic', value='/localization/pose_twist_fusion_filter/pose'),
            KeyValue(key='status', value='OK'),
            KeyValue(key='measured_rate', value='10.00 [Hz]'),
            KeyValue(key='last_message_time', value=str(self.get_clock().now().nanoseconds / 1e9))
        ]
        diag_msg.status.append(status_pose)
        self.pub_diag.publish(diag_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

