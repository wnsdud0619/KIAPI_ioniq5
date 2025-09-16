#include <pointcloud_sync.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

namespace kiapi_sensor
{

PCLSyncNode::PCLSyncNode(const rclcpp::NodeOptions & options)
: Node("kiapi_bypass_sync", options)
{
  RCLCPP_INFO(this->get_logger(), "PointCloud Sync Node Initialized");

  rclcpp::PublisherOptions pub_options;
  pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

  // Publisher: synced pointcloud
  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "concatenated/pointcloud",
    rclcpp::SensorDataQoS().keep_last(maximum_queue_size_),
    pub_options);

  // Subscribers for synchronizer
  sub_pcl_.subscribe(this, "pointcloud_before_sync", rclcpp::SensorDataQoS().get_rmw_qos_profile());
  sub_pose_.subscribe(this, "/sensing/gnss/pose_with_covariance", rclcpp::SensorDataQoS().get_rmw_qos_profile());

  // Synchronizer (ApproximateTime, queue size = 10)
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), sub_pcl_, sub_pose_);
  sync_->registerCallback(
    std::bind(&PCLSyncNode::sync_callback, this, std::placeholders::_1, std::placeholders::_2));
}

void PCLSyncNode::sync_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_msg)
{
  // keep original timestamp (important for TF)
  auto cloud_out = *cloud_msg;
  publisher_->publish(cloud_out);

  RCLCPP_INFO(this->get_logger(),
    "PointCloud Sync publish (width=%u height=%u stamp=%u.%u frame_id=%s)",
    cloud_out.width,
    cloud_out.height,
    cloud_out.header.stamp.sec,
    cloud_out.header.stamp.nanosec,
    cloud_out.header.frame_id.c_str());
}

}  // namespace kiapi_sensor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kiapi_sensor::PCLSyncNode)
