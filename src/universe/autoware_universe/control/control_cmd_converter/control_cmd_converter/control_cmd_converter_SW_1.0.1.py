#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

# Autoware Control 메시지
from autoware_control_msgs.msg import Control
# KIAPI 메시지
from my_custom_msgs.msg import ControlMessage2
from std_msgs.msg import Header
# 속도 메시지
from autoware_vehicle_msgs.msg import VelocityReport


class ControlCmdConverter(Node):
    def __init__(self):
        super().__init__('control_cmd_converter')

        # Autoware control_cmd 구독
        self.subscription = self.create_subscription(
            Control,
            '/control/trajectory_follower/control_cmd',
            self.listener_callback,
            10
        )

        # KIAPI ControlMessage2 퍼블리셔
        self.publisher = self.create_publisher(ControlMessage2, 'KIAPI/Control2/cmd_conv', 10)

        # [SH-추가] 현재 차량 속도 추적용 토픽 Subscriber
        self.current_velocity = 0.0
        self.vel_sub = self.create_subscription(
            VelocityReport,
            '/vehicle/status/velocity_status',
            self.velocity_callback,
            10
        )

        self.get_logger().info("ControlCmdConverter node started.")

    def velocity_callback(self, msg: VelocityReport):
        self.current_velocity = msg.longitudinal_velocity

    def listener_callback(self, msg: Control):
        # --- Lateral 처리 ---
        tire_angle_rad = msg.lateral.steering_tire_angle
        tire_angle_deg = tire_angle_rad * 180.0 / math.pi
        steering_wheel_deg = tire_angle_deg * 13.5  # 스티어링비 적용
        steering_wheel_deg = steering_wheel_deg

        # --- Longitudinal 처리 ---
        # 여기서는 acceleration 값을 사용 (is_defined_acceleration 확인)
        acc_cmd = msg.longitudinal.acceleration
        acc_cmd = max(min(acc_cmd, 2.0), -3.0)

        # [SH-추가] 차량 정지 상태 유지 시 사이드 브레이크 체결 방지를 위한 감속도 제한 로직
        # 감속 제어 중(acc_cmd < 0.0)이고 차량이 완전히 정지한 상태(|속도| < 0.1 = 0.36km/h)일 때
        if acc_cmd < 0.0 and abs(self.current_velocity) < 0.13: #2km/h 정도 고려함
            acc_cmd = -0.01

        # KIAPI 메시지 생성
        kiapi_msg = ControlMessage2()
        kiapi_msg.header = Header()
        kiapi_msg.header.stamp = self.get_clock().now().to_msg()

        kiapi_msg.kiapi_eps_cmd = int(steering_wheel_deg)  # 스티어링 값 실수
        kiapi_msg.kiapi_acc_cmd = int(1023 + (round(acc_cmd,2))*100)         # 가속도 값

        # 퍼블리시
        self.publisher.publish(kiapi_msg)

        # 로그 출력
        self.get_logger().info(
            f"tire_angle: {tire_angle_rad:.3f} rad "
            f"({tire_angle_deg:.2f}°) → "
            f"steering_wheel: {steering_wheel_deg:.2f}°, "
            f"acceleration: {acc_cmd:.2f} m/s²"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ControlCmdConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

