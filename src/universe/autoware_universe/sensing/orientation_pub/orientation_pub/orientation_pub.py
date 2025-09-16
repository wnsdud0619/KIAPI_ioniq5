import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from gps_msgs.msg import GPSFix
from autoware_sensing_msgs.msg import GnssInsOrientation, GnssInsOrientationStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer

class GPSIMUSyncNode(Node):
    def __init__(self):
        super().__init__('gps_imu_sync_node')
        
        # GPS와 IMU 토픽을 구독합니다.
        self.gps_sub = Subscriber(self, GPSFix, '/novatel/oem7/gps')
        self.imu_sub = Subscriber(self, Imu, '/novatel/oem7/imu/data')
        
        # 두 토픽을 싱크합니다.
        self.sync = ApproximateTimeSynchronizer([self.gps_sub, self.imu_sub], 10, 0.1)
        self.sync.registerCallback(self.callback)
        
        # 커스텀 메시지를 퍼블리시할 퍼블리셔를 생성합니다.
        self.publisher = self.create_publisher(GnssInsOrientationStamped, '/autoware_orientation', 10)

    # def callback(self, gps_msg, imu_msg):
    #     #self.get_logger().info("GPS and IMU data synced")    
    #     GnssInsOrientationStamped_msg = GnssInsOrientationStamped()
    #     GnssInsOrientation_msg = GnssInsOrientation()

    #     GnssInsOrientation_msg.orientation = imu_msg.orientation
    #     GnssInsOrientation_msg.rmse_rotation_x = gps_msg.err_roll
    #     GnssInsOrientation_msg.rmse_rotation_y = gps_msg.err_pitch
    #     GnssInsOrientation_msg.rmse_rotation_z = gps_msg.err_track
        
    #     GnssInsOrientationStamped_msg.orientation = GnssInsOrientation_msg
    #     GnssInsOrientationStamped_msg.header = gps_msg.header       
        
    #     # 커스텀 메시지 퍼블리시
    #     self.publisher.publish(GnssInsOrientationStamped_msg)
    #     #self.get_logger().info('Published custom GPS-IMU message')

    def callback(self, gps_msg, imu_msg):
        GnssInsOrientationStamped_msg = GnssInsOrientationStamped()
        GnssInsOrientation_msg = GnssInsOrientation()

        # IMU orientation quaternion
        q = imu_msg.orientation
        # Euler 각으로 변환
        roll, pitch, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # yaw -1.5도 수정 (라디안 변환)
        yaw += math.radians(-1.5)
        
        # 다시 쿼터니언으로 변환
        q_new = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        GnssInsOrientation_msg.orientation.x = q_new[0]
        GnssInsOrientation_msg.orientation.y = q_new[1]
        GnssInsOrientation_msg.orientation.z = q_new[2]
        GnssInsOrientation_msg.orientation.w = q_new[3]

        # GPS RMSE 값
        GnssInsOrientation_msg.rmse_rotation_x = gps_msg.err_roll
        GnssInsOrientation_msg.rmse_rotation_y = gps_msg.err_pitch
        GnssInsOrientation_msg.rmse_rotation_z = gps_msg.err_track
        
        GnssInsOrientationStamped_msg.orientation = GnssInsOrientation_msg
        GnssInsOrientationStamped_msg.header = gps_msg.header       
        
        self.publisher.publish(GnssInsOrientationStamped_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GPSIMUSyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

