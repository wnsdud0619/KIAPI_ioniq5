import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs

class FixFrameIdModifier(Node):
    def __init__(self):
        super().__init__('fix_frame_id_modifier')
        
        # /fix 토픽을 구독
        self.subscription = self.create_subscription(
            NavSatFix,
            '/fix',  # 원본 토픽
            self.listener_callback,
            10)
        
        # /modified_fix 토픽에 퍼블리시
        self.publisher_ = self.create_publisher(NavSatFix, '/novatel/oem7/fix', 10)
        
        # TF 브로드캐스터 초기화
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def listener_callback(self, msg):
        # NavSatFix 메시지의 header.frame_id를 변경
        msg.header.frame_id = 'gps'  # 원하는 frame_id로 변경
        
        # 입력받은 메시지의 시간으로 header.stamp 설정
        current_time = msg.header.stamp  # 메시지의 시간 사용
        
        msg.header.stamp = current_time  # 동일한 시간 사용
        
        # 변경된 메시지를 새로운 토픽에 퍼블리시
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing modified NavSatFix message with frame_id: {msg.header.frame_id} and timestamp: {msg.header.stamp}')
        
        # /fix (base_link와 gnss) 간의 변환을 게시
        self.publish_tf(current_time)

    def publish_tf(self, current_time):
        # TransformStamped 메시지 생성
        t = TransformStamped()

        # 입력받은 메시지 시간으로 header.stamp 설정
        t.header.stamp = current_time

        # 부모 및 자식 프레임 설정
        t.header.frame_id = 'base_link'  # base_link가 부모 프레임
        t.child_frame_id = 'gps'        # gnss가 자식 프레임

        # 변환값 설정 (예시: GNSS 센서 위치를 1m 앞에 두고 변환)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # 회전값 설정 (여기서는 회전 없음)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # 변환을 게시
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f'Publishing transform from base_link to gnss with timestamp: {current_time}')

def main(args=None):
    rclpy.init(args=args)
    node = FixFrameIdModifier()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

