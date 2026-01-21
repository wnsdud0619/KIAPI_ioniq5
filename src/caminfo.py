import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        
        # 1. 토픽명 /camera/camera_info, 큐 사이즈 10으로 설정
        self.publisher_ = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        
        # 2. 10Hz 주기로 타이머 설정 (0.1초)
        timer_period = 0.1  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Camera Info Publisher has been started at 10Hz')

    def timer_callback(self):
        msg = CameraInfo()
        
        # 3. 헤더 설정: 현재 시간 및 Frame ID
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        
        # 4. 카메라 파라미터 설정 (샘플 데이터 - 실제 카메라 값에 맞게 수정 필요)
        # 이미지 크기 (예: 640x480)
        msg.width = 640
        msg.height = 480
        
        # 왜곡 모델 및 계수 (D)
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # 카메라 매트릭스 (K) - 3x3 row-major
        msg.k = [500.0, 0.0, 320.0, 
                 0.0, 500.0, 240.0, 
                 0.0, 0.0, 1.0]
        
        # 투영 매트릭스 (P) - 3x4 row-major
        msg.p = [500.0, 0.0, 320.0, 0.0,
                 0.0, 500.0, 240.0, 0.0,
                 0.0, 0.0, 1.0, 0.0]

        # 5. 메시지 발행
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
