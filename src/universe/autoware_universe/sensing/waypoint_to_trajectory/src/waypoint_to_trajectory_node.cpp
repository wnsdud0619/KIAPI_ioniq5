#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <cmath>

class WaypointToTrajectory : public rclcpp::Node
{
public:
    WaypointToTrajectory() : Node("waypoint_to_trajectory"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        // Subscriber
        sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/platoon_waypoints_map", 10,
            std::bind(&WaypointToTrajectory::callback, this, std::placeholders::_1));

        // Publisher
        pub_ = this->create_publisher<autoware_planning_msgs::msg::Trajectory>(
            "/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory", 10);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
    rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    void callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        autoware_planning_msgs::msg::Trajectory traj_msg;
        const auto &poses = msg->poses;

        // 현재 차량 위치 가져오기 (map -> base_link)
        geometry_msgs::msg::TransformStamped vehicle_tf;
        double vehicle_x = 0.0, vehicle_y = 0.0;
        try {
            vehicle_tf = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
            vehicle_x = vehicle_tf.transform.translation.x;
            vehicle_y = vehicle_tf.transform.translation.y;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
            // TF 못 받아오면 첫 waypoint는 0 yaw로 설정
        }

        for (size_t i = 0; i < poses.size(); ++i)
        {
            autoware_planning_msgs::msg::TrajectoryPoint point;

            // position 그대로 사용
            point.pose.position = poses[i].position;

            // heading 계산
            double yaw = 0.0;
            if (i == 0) {
                // 첫 waypoint: 차량 현재 위치 기준
                double dx = poses[i].position.x - vehicle_x;
                double dy = poses[i].position.y - vehicle_y;
                yaw = std::atan2(dy, dx);
            }
            else if (i < poses.size() - 1) {
                // 일반 waypoint: 다음 점 기준
                double dx = poses[i + 1].position.x - poses[i].position.x;
                double dy = poses[i + 1].position.y - poses[i].position.y;
                yaw = std::atan2(dy, dx);
            }
            else {
                // 마지막 waypoint: 이전 점 기준
                double dx = poses[i].position.x - poses[i - 1].position.x;
                double dy = poses[i].position.y - poses[i - 1].position.y;
                yaw = std::atan2(dy, dx);
            }

            // quaternion 변환
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            q.normalize();
            point.pose.orientation = tf2::toMsg(q);

            // velocity 및 기타 파라미터
            point.longitudinal_velocity_mps = 16.66;
            point.lateral_velocity_mps = 0.0;
            point.acceleration_mps2 = 0.0;
            point.heading_rate_rps = 0.0;
            point.front_wheel_angle_rad = 0.0;
            point.rear_wheel_angle_rad = 0.0;

            // time_from_start 초기화
            point.time_from_start.sec = 0;
            point.time_from_start.nanosec = 0;

            traj_msg.points.push_back(point);
        }

        // publish
        pub_->publish(traj_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointToTrajectory>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

