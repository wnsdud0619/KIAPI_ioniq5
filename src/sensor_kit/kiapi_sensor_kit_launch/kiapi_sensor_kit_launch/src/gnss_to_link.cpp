#include "gnss_to_link.hpp"

GnssLinkNode::GnssLinkNode(const rclcpp::NodeOptions & options)
: Node("kiapi_gps_to_link", options)
{
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this, rclcpp::QoS(rclcpp::KeepLast(1)));

  subscription_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/sensing/gnss/pose_with_covariance", 10,
    std::bind(&GnssLinkNode::pub_tf, this, std::placeholders::_1));

  subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/novatel/oem7/imu/data", 10,
    std::bind(&GnssLinkNode::imu_callback, this, std::placeholders::_1));
}

//void GnssLinkNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
//{
//  qx_ = msg->orientation.x;
//  qy_ = msg->orientation.y;
//  qz_ = msg->orientation.z;
// qw_ = msg->orientation.w;
//}

void GnssLinkNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // 원래 쿼터니언
  tf2::Quaternion q_orig(
    msg->orientation.x,
    msg->orientation.y,
    msg->orientation.z,
    msg->orientation.w);

  // 쿼터니언 → RPY 변환
  double roll, pitch, yaw;
  tf2::Matrix3x3(q_orig).getRPY(roll, pitch, yaw);

  // yaw에 +1.5도 적용 (라디안 변환: deg * M_PI / 180.0)
  yaw -= 1.5 * M_PI / 180.0;

  // 다시 쿼터니언으로 변환
  tf2::Quaternion q_new;
  q_new.setRPY(roll, pitch, yaw);

  // 적용된 값 저장
  qx_ = q_new.x();
  qy_ = q_new.y();
  qz_ = q_new.z();
  qw_ = q_new.w();
  
  imu_received_ = true;
}

void GnssLinkNode::pub_tf(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{   
  if (!imu_received_) return;

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = msg->header.stamp;
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = "base_link";

  tf_msg.transform.translation.x = msg->pose.pose.position.x;
  tf_msg.transform.translation.y = msg->pose.pose.position.y;
  tf_msg.transform.translation.z = msg->pose.pose.position.z;

  tf_msg.transform.rotation.x = qx_;
  tf_msg.transform.rotation.y = qy_;
  tf_msg.transform.rotation.z = qz_;
  tf_msg.transform.rotation.w = qw_;

  tf_broadcaster_->sendTransform(tf_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssLinkNode>());
  rclcpp::shutdown();
  return 0;
}
