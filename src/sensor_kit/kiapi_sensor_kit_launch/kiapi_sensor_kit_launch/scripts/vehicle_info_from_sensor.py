#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from autoware_vehicle_msgs.msg import VelocityReport
from sensor_msgs.msg import Imu

### 250716 added
from novatel_oem7_msgs.msg import BESTVEL
from novatel_oem7_msgs.msg import CORRIMU

from nav_msgs.msg import Odometry


class Velinfo(Node):
    def __init__(self):
        super().__init__('kiapi_velocityReport')
        self.vel_info_msg = VelocityReport()
        self.pub = self.create_publisher(VelocityReport, '/vehicle/status/velocity_status', 10)

        print("Node Init")
        self.vel_info_msg.header.frame_id = 'base_link'
        self.timer_period = 0.01
        # self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.accu_stamp = 0.0
        self.vel_msg = VelocityReport()
        
        self.prev_time = None
        self.longitudinal_velocity = 0.0
        self.lateral_velocity = 0.0
        self.heading_rate = 0.0
        self.is_init = True
        self.count_i = 0
        
        self.sub_bestvel = self.create_subscription(BESTVEL, '/novatel/oem7/bestvel', self.BESTVELCallback, 10)
        self.sub_corrimu = self.create_subscription(CORRIMU, '/novatel/oem7/corrimu', self.CORRIMUCallback, 10)

        # self.sub_imu = self.create_subscription(Imu, '/novatel/oem7/imu/data_raw', self.ImuCallback, 10)
        # self.sub_imu = self.create_subscription(Odometry, '/novatel/oem7/odom', self.OdomCallback, 10)

    def BESTVELCallback(self, msg):
        # print("BESTVELCallback")
        # self.longitudinal_velocity = msg.hor_speed
        # self.vel_msg.header.stamp = msg.header.stamp

        vel_msg = VelocityReport()
        vel_msg.header.stamp = msg.header.stamp
        vel_msg.header.frame_id = 'base_link'
        vel_msg.longitudinal_velocity = msg.hor_speed
        vel_msg.lateral_velocity = 0.0
        vel_msg.heading_rate = self.heading_rate
        self.pub.publish(vel_msg)

    def CORRIMUCallback(self, msg):
        # print("CORRIMUCallback")
        self.heading_rate = msg.yaw_rate

    def ImuCallback(self, msg):

        cur_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time is None:
            self.prev_time = cur_time
            return
        
        dt = cur_time - self.prev_time
        self.prev_time = cur_time

        if dt <= 0:
            return  
        
        self.longitudinal_velocity += msg.linear_acceleration.x * dt
        self.lateral_velocity += msg.linear_acceleration.y * dt
        
        
        vel_msg = VelocityReport()
        vel_msg.header.stamp = msg.header.stamp
        # vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.header.frame_id = 'base_link'
        vel_msg.longitudinal_velocity = self.longitudinal_velocity
        vel_msg.lateral_velocity = self.lateral_velocity
        vel_msg.heading_rate = msg.angular_velocity.z

        self.pub.publish(vel_msg)
        print('imu:', cur_time)
        pub_time = vel_msg.header.stamp.sec + vel_msg.header.stamp.nanosec * 1e-9
        print('pub:', pub_time)
        print(cur_time - pub_time, dt)
        print()

    def OdomCallback(self, msg):
        cur_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time is None:
            self.prev_time = cur_time
            return
           
        msg.twist.twist.linear.x
        # self.longitudinal_velocity = msg.twist.twist.linear.x
        # self.lateral_velocity = msg.twist.twist.linear.y

        ### 250716 Modified
        self.lateral_velocity = 0.0

        vel_msg = VelocityReport()
        vel_msg.header.stamp = msg.header.stamp
        # vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.header.frame_id = 'base_link'
        vel_msg.longitudinal_velocity = self.longitudinal_velocity
        vel_msg.lateral_velocity = self.lateral_velocity
        vel_msg.heading_rate = msg.twist.twist.angular.z

        self.pub.publish(vel_msg)
        print('cur:', self.get_clock().now().to_msg())
        pub_time = vel_msg.header.stamp.sec + vel_msg.header.stamp.nanosec * 1e-9
        print('pub:', pub_time)
        print()

        


    def timer_callback(self):
        self.vel_info_msg.longitudinal_velocity = 0.0
        self.vel_info_msg.lateral_velocity = 0.0
        self.vel_info_msg.heading_rate = 0.0
        self.vel_info_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.vel_info_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Velinfo()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()