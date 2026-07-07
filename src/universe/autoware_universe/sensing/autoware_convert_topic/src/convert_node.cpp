#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "velodyne_msgs/msg/velodyne_scan.hpp"
#include "novatel_oem7_msgs/msg/bestvel.hpp"
#include "novatel_oem7_msgs/msg/corrimu.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_vehicle_msgs/msg/gear_report.hpp"


class ConvertNode : public rclcpp::Node
{
public:
    ConvertNode() : Node("convert_node")
    {
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/sensing/imu/novatel/oem7/imu/data_raw", 10); // /sensing/imu/tamagawa/imu_raw, /sensing/imu/novatel/oem7/imu/data_raw
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/novatel/oem7/imu/data_raw", 10,
            [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                // msg->header.frame_id = "novatel/imu_link";
                //msg->header.frame_id = "tamagawa/imu_link";
                imu_pub_->publish(*msg);
            });
       
        pcl_pub_ = this->create_publisher<velodyne_msgs::msg::VelodyneScan>("/sensing/lidar/top/velodyne_packets", 10);
        pcl_sub_ = this->create_subscription<velodyne_msgs::msg::VelodyneScan>(
            "/velodyne_packets", 10,
            [this](velodyne_msgs::msg::VelodyneScan::SharedPtr msg) {
                pcl_pub_->publish(*msg);
            });
            
        gnss_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/sensing/gnss/novatel/oem7/fix", 10); // /sensing/gnss/ublox/nav_sat_fix, /sensing/gnss/novatel/oem7/fix
        gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/novatel/oem7/fix", 10,
            [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                msg->header.frame_id = "gps_link";
                gnss_pub_->publish(*msg);
            });


        report_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", 10);
        vel_sub_ = this->create_subscription<novatel_oem7_msgs::msg::BESTVEL>(
            "/novatel/oem7/bestvel", 10,
            [this](novatel_oem7_msgs::msg::BESTVEL::SharedPtr msg) {
                vel_data_ = *msg;
            });
        corrimu_sub_ = this->create_subscription<novatel_oem7_msgs::msg::CORRIMU>(
            "/novatel/oem7/corrimu", 10,
            [this](novatel_oem7_msgs::msg::CORRIMU::SharedPtr msg) {
                // SH-로직: 수직 속도(ver_speed)가 횡방향 속도(lateral)로 들어가는 버그 수정 (0.0으로 고정)
                publishVelocityReport(vel_data_.hor_speed, 0.0, msg->yaw_rate);
            });

        // SH-NDT 후진 위치추정 로직 추가: 기어 상태를 구독하여 후진 여부 파악
        gear_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::GearReport>(
            "/vehicle/status/gear_status", 10,
            [this](autoware_vehicle_msgs::msg::GearReport::SharedPtr msg) {
                current_gear_ = msg->report;
            });

    }

private:
    void publishVelocityReport(float longitudinal_velocity, float lateral_velocity, float heading_rate)
    {
        autoware_vehicle_msgs::msg::VelocityReport report;
        report.header.stamp = vel_data_.header.stamp;
        report.header.frame_id = "base_link";

        // SH-NDT 후진 위치추정 로직 추가: 후진 기어일 경우 속도에 마이너스(-)를 붙여 차량이 뒤로 가는 것을 NDT에 올바르게 전달
        if (current_gear_ == autoware_vehicle_msgs::msg::GearReport::REVERSE || 
            current_gear_ == autoware_vehicle_msgs::msg::GearReport::REVERSE_2) {
            report.longitudinal_velocity = -std::abs(longitudinal_velocity);
        } else {
            report.longitudinal_velocity = std::abs(longitudinal_velocity);
        }

        report.lateral_velocity = lateral_velocity;
        report.heading_rate = heading_rate;

        report_pub_->publish(report);
        //RCLCPP_INFO(this->get_logger(), "Published Velocity Report: Longitudinal = %f, Lateral = %f, Heading Rate = %f",
        //            report.longitudinal_velocity, report.lateral_velocity, report.heading_rate);
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    
    rclcpp::Publisher<velodyne_msgs::msg::VelodyneScan>::SharedPtr pcl_pub_;
    rclcpp::Subscription<velodyne_msgs::msg::VelodyneScan>::SharedPtr pcl_sub_;
    
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_pub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
    
    rclcpp::Subscription<novatel_oem7_msgs::msg::BESTVEL>::SharedPtr vel_sub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr report_pub_;

    rclcpp::Subscription<novatel_oem7_msgs::msg::CORRIMU>::SharedPtr corrimu_sub_;
    novatel_oem7_msgs::msg::BESTVEL vel_data_;

    rclcpp::Subscription<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_sub_;
    uint8_t current_gear_ = autoware_vehicle_msgs::msg::GearReport::NONE;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ConvertNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
