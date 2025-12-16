#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <autoware_map_msgs/msg/map_projector_info.hpp>
#include <autoware/geography_utils/height.hpp>
#include <autoware/geography_utils/projection.hpp>
#include "kiapi_waypoints_msgs/msg/kiapi_waypoints.hpp"

class PlatoonWaypointPoser : public rclcpp::Node
{
public:
  PlatoonWaypointPoser()
  : Node("platoon_waypoint_poser"), received_map_projector_info_(false)
  {
    // Map Projector Info 구독
    auto qos_map_info = rclcpp::QoS(rclcpp::KeepLast(1));
    qos_map_info.reliable();
    qos_map_info.transient_local();
    map_projector_info_sub_ = this->create_subscription<autoware_map_msgs::msg::MapProjectorInfo>(
      "/map/map_projector_info", qos_map_info,
      std::bind(&PlatoonWaypointPoser::callbackMapProjectorInfo, this, std::placeholders::_1));

    // Waypoints 구독
    waypoints_sub_ = this->create_subscription<kiapi_waypoints_msgs::msg::KiapiWaypoints>(
      "/KIAPI_waypoints", rclcpp::QoS(10),
      std::bind(&PlatoonWaypointPoser::callbackWaypoints, this, std::placeholders::_1));

    // Marker 퍼블리셔
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/platoon_waypoints_marker", rclcpp::QoS(10));

    // 변환된 좌표 PoseArray 퍼블리셔
    converted_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/platoon_waypoints_map", rclcpp::QoS(10));

    RCLCPP_INFO(this->get_logger(), "Platoon Waypoint Poser Node Started ✅");
  }

private:
  void callbackMapProjectorInfo(
    const autoware_map_msgs::msg::MapProjectorInfo::ConstSharedPtr msg)
  {
    projector_info_ = *msg;
    if (!received_map_projector_info_) {
      RCLCPP_INFO(this->get_logger(),
                  "Received /map/map_projector_info once:\n"
                  "  projector_type: %s\n"
                  "  vertical_datum: %s\n"
                  "  map_origin(lat,lon,alt): %.8f, %.8f, %.2f",
                  msg->projector_type.c_str(),
                  msg->vertical_datum.c_str(),
                  msg->map_origin.latitude,
                  msg->map_origin.longitude,
                  msg->map_origin.altitude);
      received_map_projector_info_ = true;
    }
  }

  void callbackWaypoints(
    const kiapi_waypoints_msgs::msg::KiapiWaypoints::ConstSharedPtr msg)
  {
    if (!received_map_projector_info_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Waiting for /map/map_projector_info...");
      return;
    }

    // PoseArray와 Marker 초기화
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = "map";
    pose_array.header.stamp = this->now();

    visualization_msgs::msg::Marker marker;
    marker.header = pose_array.header;
    marker.ns = "platoon_waypoints";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.8;
    marker.color.b = 1.0;
    marker.lifetime = rclcpp::Duration(0,0);

    const double yaw = msg->heading_rad;
    const double half = 0.5 * yaw;
    const double qz = std::sin(half);
    const double qw = std::cos(half);

    // Waypoint 변환
    for (const auto & gp : msg->gps_point) {
      try {
        geometry_msgs::msg::Point map_point =
          autoware::geography_utils::project_forward(gp, projector_info_);

        map_point.z = autoware::geography_utils::convert_height(
          map_point.z, gp.latitude, gp.longitude,
          autoware_map_msgs::msg::MapProjectorInfo::WGS84,
          projector_info_.vertical_datum);

        marker.points.push_back(map_point);

        geometry_msgs::msg::Pose pose;
        pose.position = map_point;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = qz;
        pose.orientation.w = qw;
        pose_array.poses.push_back(pose);
      }
      catch (const std::exception & e) {
        RCLCPP_WARN(this->get_logger(),
                    "Skipping waypoint due to exception: %s", e.what());
        continue;
      }
    }

    // Publish
    marker_pub_->publish(marker);
    converted_pub_->publish(pose_array);
  }

  rclcpp::Subscription<kiapi_waypoints_msgs::msg::KiapiWaypoints>::SharedPtr waypoints_sub_;
  rclcpp::Subscription<autoware_map_msgs::msg::MapProjectorInfo>::SharedPtr map_projector_info_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr converted_pub_;
  autoware_map_msgs::msg::MapProjectorInfo projector_info_;
  bool received_map_projector_info_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlatoonWaypointPoser>());
  rclcpp::shutdown();
  return 0;
}

