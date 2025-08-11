#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <QLabel>
#include <QVBoxLayout>
#include <QTimer>
#include <QMetaObject>
#include "novatel_oem7_msgs/msg/bestpos.hpp"

namespace rviz_text_panel
{

class TextPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit TextPanel(QWidget* parent = nullptr)
  : rviz_common::Panel(parent)
  {  
    qRegisterMetaType<uint32_t>("uint32_t");
    label_ = new QLabel("Waiting for GPS data...");
    label_->setAlignment(Qt::AlignCenter);

    auto layout = new QVBoxLayout;
    layout->addWidget(label_);
    setLayout(layout);

    node_ = rclcpp::Node::make_shared("gps_status_panel_node");

    subscription_ = node_->create_subscription<novatel_oem7_msgs::msg::BESTPOS>(
      "/novatel/oem7/bestpos", 10,
      [this](novatel_oem7_msgs::msg::BESTPOS::UniquePtr msg) {
        // Qt 메인스레드에서 updateStatus 호출
        QMetaObject::invokeMethod(this, "updateStatus",
            Qt::QueuedConnection,
            Q_ARG(uint32_t, msg->pos_type.type));
      });

    // 타이머로 rclcpp spin_some 호출 (ROS 메시지 콜백 처리용)
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &TextPanel::spinOnce);
    timer_->start(50);  // 20Hz
  }

public Q_SLOTS:
void updateStatus(uint32_t pos_type)
{
  QString status;
  QString color;

  if (pos_type == 0) {
    status = "FAIL";
    color = "red";  // 빨간색
  }
  else if (pos_type >= 16 && pos_type <= 20) {
    status = "SINGLE";
    color = "#90EE90";  // 연한 초록 (LightGreen)
  }
  else if (pos_type >= 32) {
    status = "RTK";
    color = "green";  // 초록색
  }
  else {
    status = "UNKNOWN";
    color = "yellow";  // 노란색
  }

  label_->setText("GPS 상태: " + status);

  // 텍스트 색상 변경 (스타일 시트 적용)
  label_->setStyleSheet(QString("QLabel { color : %1; }").arg(color));
}

private Q_SLOTS:
  void spinOnce()
  {
    rclcpp::spin_some(node_);
  }

private:
  QLabel* label_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::BESTPOS>::SharedPtr subscription_;
  QTimer* timer_;
};

}  // namespace rviz_text_panel
Q_DECLARE_METATYPE(uint32_t)

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_text_panel::TextPanel, rviz_common::Panel)
#include "text_panel.moc"

