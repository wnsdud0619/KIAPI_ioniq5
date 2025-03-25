#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "velodyne_msgs/msg/velodyne_scan.hpp"
#include "novatel_oem7_msgs/msg/bestvel.hpp"
#include "novatel_oem7_msgs/msg/corrimu.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"


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
        corrimu_sub_ = this->create_subscription<novatel_oem7_msgs::msg::CORRIMU>(
            "/novatel/oem7/corrimu", 10,
            [this](novatel_oem7_msgs::msg::CORRIMU::SharedPtr msg) {
                publishVelocityReport(vel_data_.hor_speed, msg->yaw_rate);
            });
        vel_sub_ = this->create_subscription<novatel_oem7_msgs::msg::BESTVEL>(
            "/novatel/oem7/bestvel", 10,
            [this](novatel_oem7_msgs::msg::BESTVEL::SharedPtr msg) {
                vel_data_ = *msg;
            });
    }

private:
    void publishVelocityReport(float longitudinal_velocity, float heading_rate)
    {
        autoware_vehicle_msgs::msg::VelocityReport report;
        report.header.stamp = vel_data_.header.stamp;
        report.header.frame_id = "base_link";
        report.longitudinal_velocity = longitudinal_velocity;
        report.lateral_velocity = 0.0;
        report.heading_rate = heading_rate;

        report_pub_->publish(report);
        RCLCPP_INFO(this->get_logger(), "Published Velocity Report: Longitudinal = %f, Lateral = %f, Heading Rate = %f",
                    report.longitudinal_velocity, report.lateral_velocity, report.heading_rate);
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
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ConvertNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
