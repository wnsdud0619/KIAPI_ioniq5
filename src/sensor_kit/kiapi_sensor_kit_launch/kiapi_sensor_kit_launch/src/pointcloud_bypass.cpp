#include <pointcloud_bypass.hpp>

using namespace std::chrono_literals;

namespace kiapi_sensor
{

PCLBypassNode::PCLBypassNode(const rclcpp::NodeOptions & options)
: Node("kiapi_bypass", options)
{
  RCLCPP_INFO(this->get_logger(), "PointCloud Sync Node Initialized");

  rclcpp::PublisherOptions pub_options;
  pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

  // Publisher: synced pointcloud
  publisher_ =  this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "concatenated/pointcloud",
    rclcpp::SensorDataQoS().keep_last(maximum_queue_size_),
    pub_options);

  // Subscribers
  subscription_ =  this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud_raw_ex", 
    rclcpp::SensorDataQoS().keep_last(maximum_queue_size_),
    std::bind(&PCLBypassNode::convert_point_xyzirc, this, std::placeholders::_1));
  
}

void PCLBypassNode::convert_point_xyzirc(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{ 

  const size_t point_count = msg->width * msg->height;

  sensor_msgs::msg::PointCloud2 output;
  output.header = msg->header;
  output.height = 1;
  output.width = point_count;
  output.is_bigendian = false;
  output.is_dense = true;
  output.point_step = 16; 
  output.row_step = output.point_step * output.width;
  output.data.resize(output.row_step);

  sensor_msgs::msg::PointField field;
  field.name = "x";
  field.offset = 0;
  field.datatype = 7;
  field.count = 1;
  output.fields.push_back(field);

  field.name = "y";
  field.offset = 4;
  field.datatype = 7;
  field.count = 1;
  output.fields.push_back(field);

  field.name = "z";
  field.offset = 8;
  field.datatype = 7;
  field.count = 1;
  output.fields.push_back(field);

  field.name = "intensity";
  field.offset = 12;
  field.datatype = 2;
  field.count = 1;
  output.fields.push_back(field);

  field.name = "return_type";
  field.offset = 13;
  field.datatype = 2;
  field.count = 1;
  output.fields.push_back(field);

  field.name = "channel";
  field.offset = 14;
  field.datatype = 4;
  field.count = 1;
  output.fields.push_back(field);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_intensity(*msg, "intensity");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_return_type(*msg, "return_type");
  sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_channel(*msg, "channel");

  for (size_t i = 0; i < point_count;
        ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_return_type, ++iter_channel)
  {
    uint8_t * ptr = &output.data[i * output.point_step];

    *reinterpret_cast<float*>(ptr + 0) = *iter_x;
    *reinterpret_cast<float*>(ptr + 4) = *iter_y;
    *reinterpret_cast<float*>(ptr + 8) = *iter_z;
    *(ptr + 12) = *iter_intensity;
    *(ptr + 13) = *iter_return_type;
    *reinterpret_cast<uint16_t*>(ptr + 14) = *iter_channel;  
  }
  output.header.frame_id = "base_link";
  output.header.stamp = this->get_clock()->now();

  publisher_->publish(output);
}
  

}  // namespace kiapi_sensor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kiapi_sensor::PCLBypassNode)
