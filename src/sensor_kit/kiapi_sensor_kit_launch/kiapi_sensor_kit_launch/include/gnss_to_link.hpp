#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class GnssLinkNode : public rclcpp::Node
{
public:
  explicit GnssLinkNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_pose_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu_;

  // TF broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Quaternion storage
  double qx_ = 0.0, qy_ = 0.0, qz_ = 0.0, qw_ = 1.0;

  // Callbacks
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void pub_tf(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  
  bool imu_received_ = false;  
};
