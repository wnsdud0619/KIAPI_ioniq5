#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        
        # Autoware Sensing 레이어 표준 토픽명으로 설정
        # 필요에 따라 '/perception/traffic_light_recognition/camera/camera_info'로 변경 가능
        self.publisher_ = self.create_publisher(
            CameraInfo, 
            '/camera/camera_info', 
            10
        )
        
        # 10Hz 주기로 타이머 설정 (0.1초)
        timer_period = 0.1  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Camera Info Publisher (1920x1080) started.')
        self.get_logger().info('Topic: /sensing/camera/camera/camera_info')

    def timer_callback(self):
        msg = CameraInfo()
        
        # 1. 헤더 설정
        # 실행 시 use_sim_time:=true를 주면 Rosbag 시간을 자동으로 가져옴
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_optical_link'
        
        # 2. 이미지 크기 (Full HD)
        msg.width = 1920
        msg.height = 1080
        
        # 3. 왜곡 모델 (기본값)
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # 4. 카메라 매트릭스 (K) - 1920x1080 비율에 맞춤
        # [fx,  0, cx]
        # [ 0, fy, cy]
        # [ 0,  0,  1]
        # fx, fy: 초점 거리 (픽셀 단위). 1500 내외가 일반적
        # cx, cy: 주점 (이미지 중심). 1920/2=960, 1080/2=540
        msg.k = [1500.0,    0.0,  960.0, 
                    0.0, 1500.0,  540.0, 
                    0.0,    0.0,    1.0]
        
        # 5. Rectification 매트릭스 (R) - 단위 행렬 (필수)
        msg.r = [1.0, 0.0, 0.0, 
                 0.0, 1.0, 0.0, 
                 0.0, 0.0, 1.0]
        
        # 6. 투영 매트릭스 (P) - 3x4
        # [fx',  0, cx', Tx]
        # [ 0, fy', cy', Ty]
        # [ 0,  0,  1,   0]
        msg.p = [1500.0,    0.0,  960.0,  0.0,
                    0.0, 1500.0,  540.0,  0.0,
                    0.0,    0.0,    1.0,  0.0]

        # 7. 메시지 발행
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
