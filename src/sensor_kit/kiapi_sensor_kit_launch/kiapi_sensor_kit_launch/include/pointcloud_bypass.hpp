#ifndef KIAPI_SENSOR_KIT_LAUNCH__POINTCLOUD_BYPASS_HPP_
#define KIAPI_SENSOR_KIT_LAUNCH__POINTCLOUD_BYPASS_HPP_

// ROS2 Components
#include "visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_field.hpp>


// Nebula Components
// #include "autoware/point_types/types.hpp"
#include <boost/tokenizer.hpp>

#include <algorithm>
#include <ostream>
#include <string>
#include <memory>
#include <vector>
#include <chrono>
#include <functional>
#include <cmath>
#include <tuple>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


struct EIGEN_ALIGN16 PointXYZIRC
{
  PCL_ADD_POINT4D;
  uint8_t intensity;             
  uint8_t return_type;         
  uint8_t channel;         
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRC,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (std::uint8_t, intensity, intensity)
  (std::uint8_t, return_type, return_type)
  (std::uint16_t, channel, channel))



namespace kiapi_sensor
{

class PCLBypassNode : public rclcpp::Node
{
public:
  KIAPI_SENSOR_KIT_LAUNCH_PUBLIC
  explicit PCLBypassNode(const rclcpp::NodeOptions & options);
  sensor_msgs::msg::PointCloud2 synced_msg;
  sensor_msgs::msg::PointField pointfield_msg;
  int data_length;

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  void convert_point_xyzirc(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /// @brief Converts degrees to radians
  /// @param radians
  /// @return degrees
  static inline float deg2rad(double degrees)
  {
    return degrees * M_PI / 180.0;
  }

  /// @brief Converts radians to degrees
  /// @param radians
  /// @return degrees
  static inline float rad2deg(double radians)
  {
    return radians * 180.0 / M_PI;
  }
  // queue size parameter
  int64_t maximum_queue_size_ = 10;
};

}  // namespace kiapi_sensor

#endif  // KIAPI_SENSOR_KIT_LAUNCH__POINTCLOUD_BYPASS_HPP_
