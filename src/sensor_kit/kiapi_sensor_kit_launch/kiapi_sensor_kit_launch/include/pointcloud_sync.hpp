#ifndef KIAPI_SENSOR_KIT_LAUNCH__POINTCLOUD_SYNC_HPP_
#define KIAPI_SENSOR_KIT_LAUNCH__POINTCLOUD_SYNC_HPP_

#include "visibility_control.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace kiapi_sensor
{

class PCLSyncNode : public rclcpp::Node
{
public:
  KIAPI_SENSOR_KIT_LAUNCH_PUBLIC
  explicit PCLSyncNode(const rclcpp::NodeOptions & options);

private:
  // Synchronizer policy: PointCloud2 + Pose
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2,
    geometry_msgs::msg::PoseWithCovarianceStamped>;

  // sync callback
  void sync_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg,
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_msg);

  // publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

  // message_filter subscribers
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_pcl_;
  message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped> sub_pose_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // queue size parameter
  int64_t maximum_queue_size_ = 10;
};

}  // namespace kiapi_sensor

#endif  // KIAPI_SENSOR_KIT_LAUNCH__POINTCLOUD_SYNC_HPP_
