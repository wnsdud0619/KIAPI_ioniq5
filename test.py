import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros

class GpsTfBroadcaster(Node):
    def __init__(self):
        super().__init__('gps_tf_broadcaster')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.broadcast_tf)  # 0.01초(100Hz)마다 실행

    def broadcast_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'gps'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 0.0  # GNSS 센서의 오프셋에 맞게 조정
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0  # 회전 없음

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = GpsTfBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#test
