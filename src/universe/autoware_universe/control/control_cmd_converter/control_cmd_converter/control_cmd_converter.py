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
        self.is_stopped = False  # 정지 상태를 기억하는 상태 변수

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

        # --- Longitudinal 처리 ---
        # 여기서는 acceleration 값을 사용 (is_defined_acceleration 확인)
        acc_cmd = msg.longitudinal.acceleration
        acc_cmd = max(min(acc_cmd, 2.0), -3.0)

        # [SH-추가] 멈출 때 꿀렁임(감속:-3.0) 방지 및 부드러운 정차를 위한 감속도 제어 최적화 로직
        v_current = abs(self.current_velocity)

        # 1. 출발 조건 (멈춰있다가 Autoware가 양수(+)의 가속을 명령할 때)
        if acc_cmd > 0.0:
            self.is_stopped = False  # 정지 상태 해제, 가속 명령 그대로 통과
            
        # 2. 감속 및 정지 제어 로직 (acc_cmd <= 0.0)
        else:
            if self.is_stopped:
                # [상태 확인] 이미 완전히 정지했다고 판단했다면, 
                # 노이즈로 인해 속도가 잠시 튀더라도 가속(가속도>0) 전까지는 무조건 -0.01 유지 
                acc_cmd = -0.01
            else:
                # 아직 정차하는 중일 때: 속도에 비례해서 감속도 조절+ 
                v_blend_start = 0.6  # 약 2.8km/h: 이 속도부터 감속도를 부드럽게 내리기 시작
                v_blend_end = 0.005    # 약 0.7km/h: 이 속도 이하가 되면 완전 정지로 간주

                if v_current <= v_blend_end:
                    self.is_stopped = True  # 완전 정지 상태 진입
                    acc_cmd = -0.01
                elif v_current < v_blend_start:
                    # v_blend_start(0.8) ~ v_blend_end(0.2) 구간 보간법 적용
                    # v_current가 0.2에 가까워질수록 weight는 0이 되어 -0.01의 비중이 커짐
                    weight = (v_current - v_blend_end) / (v_blend_start - v_blend_end)
                    acc_cmd = (weight * acc_cmd) + ((1.0 - weight) * -0.01)

        # KIAPI 메시지 생성
        kiapi_msg = ControlMessage2()
        kiapi_msg.header = Header()
        kiapi_msg.header.stamp = self.get_clock().now().to_msg()

        kiapi_msg.kiapi_eps_cmd = int(steering_wheel_deg)
        kiapi_msg.kiapi_acc_cmd = int(1023 + (round(acc_cmd, 2)) * 100)

        # 퍼블리시
        self.publisher.publish(kiapi_msg)

        # 로그 출력
        self.get_logger().info(
            f"tire_angle: {tire_angle_rad:.3f} rad "
            f"({tire_angle_deg:.2f}°) → "
            f"steering_wheel: {steering_wheel_deg:.2f}°, "
            f"acceleration: {acc_cmd:.2f} m/s² | is_stopped: {self.is_stopped}"
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