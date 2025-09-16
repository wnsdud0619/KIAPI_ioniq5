#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "velodyne_msgs/msg/velodyne_scan.hpp"
#include "novatel_oem7_msgs/msg/bestvel.hpp"
#include "novatel_oem7_msgs/msg/corrimu.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"

// ★ 추가: GPSFix / Autoware sensing msgs / message_filters
#include "gps_msgs/msg/gps_fix.hpp"
#include "autoware_sensing_msgs/msg/gnss_ins_orientation.hpp"
#include "autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class ConvertNode : public rclcpp::Node
{
public:
  ConvertNode() : Node("convert_node")
  {
    // --------------- 기존 패스스루/변환 퍼블리셔 & 구독 ----------------
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "/sensing/imu/novatel/oem7/imu/data_raw", 10);

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/novatel/oem7/imu/data_raw", 10,
        [this](sensor_msgs::msg::Imu::SharedPtr msg) {
          imu_pub_->publish(*msg);
        });

    pcl_pub_ = this->create_publisher<velodyne_msgs::msg::VelodyneScan>(
        "/sensing/lidar/top/velodyne_packets", 10);

    pcl_sub_ = this->create_subscription<velodyne_msgs::msg::VelodyneScan>(
        "/velodyne_packets", 10,
        [this](velodyne_msgs::msg::VelodyneScan::SharedPtr msg) {
          pcl_pub_->publish(*msg);
        });

    gnss_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
        "/sensing/gnss/novatel/oem7/fix", 10);

    gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/novatel/oem7/fix", 10,
        [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) {
          gnss_pub_->publish(*msg);
        });

    report_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
        "/vehicle/status/velocity_status", 10);

    corrimu_sub_ = this->create_subscription<novatel_oem7_msgs::msg::CORRIMU>(
        "/novatel/oem7/corrimu", 10,
        [this](novatel_oem7_msgs::msg::CORRIMU::SharedPtr msg) {
          publishVelocityReport(vel_data_.hor_speed, vel_data_.ver_speed, msg->yaw_rate);
        });

    vel_sub_ = this->create_subscription<novatel_oem7_msgs::msg::BESTVEL>(
        "/novatel/oem7/bestvel", 10,
        [this](novatel_oem7_msgs::msg::BESTVEL::SharedPtr msg) {
          vel_data_ = *msg;
        });

    // --------------- (신규) GPSFix + IMU ApproximateTime 동기화 ----------------
    gps_topic_  = this->declare_parameter<std::string>("gpsfix_topic", "/novatel/oem7/gps");
    imu_topic_  = this->declare_parameter<std::string>("imu_topic",    "/novatel/oem7/imu/data");
    out_topic_  = this->declare_parameter<std::string>("out_topic",    "/autoware_orientation");
    sync_queue_ = this->declare_parameter<int>("sync_queue_size", 10);
    sync_slop_  = this->declare_parameter<double>("sync_slop_sec", 0.10); // 100ms

    gnssins_pub_ = this->create_publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>(
        out_topic_, 10);

    // message_filters 구독자
    gps_sub_.subscribe(this, gps_topic_);
    imu_sync_sub_.subscribe(this, imu_topic_);

    // 정책 & 동기화기 생성
    sync_ = std::make_shared<Sync>(SyncPolicy(sync_queue_), gps_sub_, imu_sync_sub_);
    sync_->registerCallback(std::bind(&ConvertNode::syncCallback, this,
                                      std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
                "Syncing GPSFix(%s) + Imu(%s) -> %s | queue=%d, slop=%.2fs",
                gps_topic_.c_str(), imu_topic_.c_str(), out_topic_.c_str(), sync_queue_, sync_slop_);
  }

private:
  void publishVelocityReport(float longitudinal_velocity, float lat_velocity, float heading_rate)
  {
    autoware_vehicle_msgs::msg::VelocityReport report;
    report.header.stamp = vel_data_.header.stamp;
    report.header.frame_id = "base_link";
    report.longitudinal_velocity = longitudinal_velocity;
    report.lateral_velocity = lat_velocity;
    report.heading_rate = heading_rate;

    report_pub_->publish(report);
    //RCLCPP_INFO(this->get_logger(),
    //           "Published Velocity Report: Longitudinal = %f, Lateral = %f, Heading Rate = %f",
    //            report.longitudinal_velocity, report.lateral_velocity, report.heading_rate);
  }

  // ★ 동기 콜백: Python 예제와 동일 컨셉
  void syncCallback(const gps_msgs::msg::GPSFix::ConstSharedPtr &gps_msg,
                    const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)
  {
    // (선택) slop 체크: message_filters가 slop 파라미터를 직접 받지 않으므로 수동 필터링
    const double tg = gps_msg->header.stamp.sec + 1e-9 * gps_msg->header.stamp.nanosec;
    const double ti = imu_msg->header.stamp.sec + 1e-9 * imu_msg->header.stamp.nanosec;
    if (std::abs(tg - ti) > sync_slop_) {
      // 과도한 차이는 무시(로그는 과다 방지를 위해 생략/주석)
      // RCLCPP_WARN(this->get_logger(), "Pair outside slop: |%.3f-%.3f|=%.3f > %.3f", tg, ti, std::abs(tg-ti), sync_slop_);
      return;
    }

    autoware_sensing_msgs::msg::GnssInsOrientationStamped out;
    autoware_sensing_msgs::msg::GnssInsOrientation ori;

    // IMU orientation
    ori.orientation = imu_msg->orientation;

    // GPSFix의 각도 RMSE (단위는 드라이버 정의에 따름: rad/deg 확인 필요)
    ori.rmse_rotation_x = gps_msg->err_roll;
    ori.rmse_rotation_y = gps_msg->err_pitch;
    ori.rmse_rotation_z = gps_msg->err_track;

    out.orientation = ori;
    out.header = gps_msg->header;  // 필요 시 imu_msg->header 사용으로 변경 가능

    gnssins_pub_->publish(out);
  }

  // ---------------- 멤버 ----------------
  // 기존 퍼블/섭
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

  // 동기화 관련
  std::string gps_topic_, imu_topic_, out_topic_;
  int    sync_queue_{10};
  double sync_slop_{0.10};

  // message_filters 구독자
  message_filters::Subscriber<gps_msgs::msg::GPSFix> gps_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Imu>  imu_sync_sub_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      gps_msgs::msg::GPSFix, sensor_msgs::msg::Imu>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;
  std::shared_ptr<Sync> sync_;

  // GnssInsOrientation 퍼블리셔
  rclcpp::Publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>::SharedPtr gnssins_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ConvertNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
