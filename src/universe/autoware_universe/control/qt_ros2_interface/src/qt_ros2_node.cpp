#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <QApplication>
#include <QPushButton>
#include <QWidget>
#include <QVBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QSlider>
#include <QLineEdit>
#include <my_custom_msgs/msg/control_message1.hpp>
#include <my_custom_msgs/msg/control_message2.hpp>

class QtRos2Node : public QWidget
{
    Q_OBJECT

public:
    QtRos2Node(rclcpp::Node::SharedPtr node, QWidget *parent = nullptr)
        : QWidget(parent), node_(node), eps_en_(false), controlsw_(false), scc_en_(false), turn_sig_L_(false), turn_sig_R_(false), eps_cmd_(0), acc_cmd_(0), target_speed_(0)
    {
        button_eps_en = new QPushButton("Toggle EPS_En", this);
        button_controlsw = new QPushButton("Toggle ControlSW", this);
        button_scc_en = new QPushButton("Toggle SCC_En", this);
        button_turn_sig_L = new QPushButton("Toggle turn_sig_L", this);
        button_turn_sig_R = new QPushButton("Toggle turn_sig_R", this);

        slider_eps_cmd = new QSlider(Qt::Horizontal, this);
        slider_eps_cmd->setRange(0, 10000);
        slider_eps_cmd->setValue(5000);

        label_eps_cmd = new QLabel("EPS_Cmd: 0", this);

        slider_acc_cmd = new QSlider(Qt::Horizontal, this);
        slider_acc_cmd->setRange(0, 400);
        slider_acc_cmd->setValue(300);

        label_acc_cmd = new QLabel("ACC_Cmd: 0", this);

        line_edit_eps_cmd = new QLineEdit(this);
        line_edit_eps_cmd->setPlaceholderText("Enter EPS_Cmd value");
        line_edit_eps_cmd->setText(QString::number(eps_cmd_));

        line_edit_acc_cmd = new QLineEdit(this);
        line_edit_acc_cmd->setPlaceholderText("Enter ACC_Cmd value");
        line_edit_acc_cmd->setText(QString::number(acc_cmd_));

        eps_min_ = -5000;
        eps_max_ = 5000;
        acc_min_ = -300;
        acc_max_ = 100;

        // Target Speed 추가
        slider_target_speed = new QSlider(Qt::Horizontal, this);
        slider_target_speed->setRange(0, 100);
        slider_target_speed->setValue(0);

        label_target_speed = new QLabel("Target Speed: 0 km/h", this);

        line_edit_target_speed = new QLineEdit(this);
        line_edit_target_speed->setPlaceholderText("Enter target speed (0~100)");
        line_edit_target_speed->setText(QString::number(target_speed_));

        QVBoxLayout *layout = new QVBoxLayout;
        layout->addWidget(new QLabel("Press the buttons to toggle values"));
        layout->addWidget(button_controlsw);
        layout->addWidget(button_eps_en);
        layout->addWidget(button_scc_en);
        layout->addWidget(button_turn_sig_L);
        layout->addWidget(button_turn_sig_R);
        layout->addWidget(slider_eps_cmd);
        layout->addWidget(label_eps_cmd);
        layout->addWidget(line_edit_eps_cmd);
        layout->addWidget(slider_acc_cmd);
        layout->addWidget(label_acc_cmd);
        layout->addWidget(line_edit_acc_cmd);
        layout->addWidget(slider_target_speed);
        layout->addWidget(label_target_speed);
        layout->addWidget(line_edit_target_speed);

        setLayout(layout);

        connect(button_controlsw, &QPushButton::clicked, this, &QtRos2Node::toggleControlSW);
        connect(button_eps_en, &QPushButton::clicked, this, &QtRos2Node::toggleEPS_En);
        connect(button_scc_en, &QPushButton::clicked, this, &QtRos2Node::toggleSCC_En);
        connect(button_turn_sig_L, &QPushButton::clicked, this, &QtRos2Node::toggleTurnSig_L);
        connect(button_turn_sig_R, &QPushButton::clicked, this, &QtRos2Node::toggleTurnSig_R);

        connect(slider_eps_cmd, &QSlider::valueChanged, this, &QtRos2Node::updateEPS_Cmd);
        connect(slider_acc_cmd, &QSlider::valueChanged, this, &QtRos2Node::updateACC_Cmd);
        connect(line_edit_eps_cmd, &QLineEdit::textChanged, this, &QtRos2Node::updateEPSCmdFromLineEdit);
        connect(line_edit_acc_cmd, &QLineEdit::textChanged, this, &QtRos2Node::updateACC_CmdFromLineEdit);

        connect(slider_target_speed, &QSlider::valueChanged, this, &QtRos2Node::updateTargetSpeed);
        connect(line_edit_target_speed, &QLineEdit::textChanged, this, &QtRos2Node::updateTargetSpeedFromLineEdit);

        setFixedSize(300, 500);

        setupPublisher();

        message_timer_ = new QTimer(this);
        connect(message_timer_, &QTimer::timeout, this, &QtRos2Node::publishMessage);
        message_timer_->start(10);

        setOtherButtonsEnabled(false);
        setEPSSliderEnabled(false);
        setACCSliderEnabled(false);
        setTargetSpeedEnabled(false);
    }

public Q_SLOTS:
    void toggleControlSW() {
        controlsw_ = !controlsw_;
        setOtherButtonsEnabled(controlsw_);
        setEPSSliderEnabled(controlsw_ && eps_en_);
        setACCSliderEnabled(controlsw_ && scc_en_);
    }

    void toggleEPS_En() {
        if (controlsw_) {
            eps_en_ = !eps_en_;
            setEPSSliderEnabled(controlsw_ && eps_en_);
        }
    }

    void toggleSCC_En() {
        if (controlsw_) {
            scc_en_ = !scc_en_;
            setACCSliderEnabled(controlsw_ && scc_en_);
            setTargetSpeedEnabled(controlsw_ && scc_en_);
        }
    }

    void toggleTurnSig_L() { if (controlsw_) turn_sig_L_ = !turn_sig_L_; }
    void toggleTurnSig_R() { if (controlsw_) turn_sig_R_ = !turn_sig_R_; }

    void updateEPS_Cmd(int value) {
        eps_cmd_ = value - 5000;
        label_eps_cmd->setText(QString("EPS_Cmd: %1").arg(eps_cmd_ * 0.1));
    }

    void updateACC_Cmd(int value) {
        acc_cmd_ = value - 300;
        label_acc_cmd->setText(QString("EPS_Cmd: %1").arg(acc_cmd_ * 0.01));
    }

    void updateEPSCmdFromLineEdit() {
        bool ok;
        double value = line_edit_eps_cmd->text().toDouble(&ok);
        if (ok && value >= eps_min_ && value <= eps_max_) {
            eps_cmd_ = static_cast<int>(value);
            label_eps_cmd->setText(QString("EPS_Cmd: %1").arg(float(value * 0.1), 0, 'f', 1));
            slider_eps_cmd->setValue(eps_cmd_ + 5000);
        } else if (ok) {
            line_edit_eps_cmd->setText(QString::number(eps_cmd_));
        }
    }

    void updateACC_CmdFromLineEdit() {
        bool ok;
        double value = line_edit_acc_cmd->text().toDouble(&ok);
        if (ok && value >= acc_min_ && value <= acc_max_) {
            acc_cmd_ = static_cast<int>(value);
            label_acc_cmd->setText(QString("ACC_Cmd: %1").arg(float(value * 0.01), 0, 'f', 1));
            slider_acc_cmd->setValue(acc_cmd_ + 300);
        } else if (ok) {
            line_edit_acc_cmd->setText(QString::number(acc_cmd_));
        }
    }

    void updateTargetSpeed(int value) {
        target_speed_ = value;
        label_target_speed->setText(QString("Target Speed: %1 km/h").arg(target_speed_));
        line_edit_target_speed->setText(QString::number(target_speed_));
    }

    void updateTargetSpeedFromLineEdit() {
        bool ok;
        int value = line_edit_target_speed->text().toInt(&ok);
        if (ok && value >= 0 && value <= 100) {
            target_speed_ = value;
            label_target_speed->setText(QString("Target Speed: %1 km/h").arg(target_speed_));
            slider_target_speed->setValue(target_speed_);
        } else if (ok) {
            line_edit_target_speed->setText(QString::number(target_speed_));
        }
    }

    void publishMessage() {
        auto ros_message = my_custom_msgs::msg::ControlMessage1();
        ros_message.header.stamp = node_->now();
        ros_message.header.frame_id = "KIAPI_frame_id";
        ros_message.controlsw = controlsw_;
        ros_message.eps_en = eps_en_;
        ros_message.scc_en = scc_en_;
        ros_message.turn_sig_l = turn_sig_L_;
        ros_message.turn_sig_r = turn_sig_R_;
        publisher_->publish(ros_message);

        auto ros_message2 = my_custom_msgs::msg::ControlMessage2();
        ros_message2.header.stamp = node_->now();
        ros_message2.header.frame_id = "KIAPI_frame_id";
        ros_message2.eps_cmd = eps_cmd_;
        ros_message2.acc_cmd = acc_cmd_ + 1023;
        ros_message2.target_speed = target_speed_;
        publisher2_->publish(ros_message2);
    }

    void setOtherButtonsEnabled(bool enabled) {
        button_eps_en->setEnabled(enabled);
        button_scc_en->setEnabled(enabled);
        button_turn_sig_L->setEnabled(enabled);
        button_turn_sig_R->setEnabled(enabled);
    }

    void setEPSSliderEnabled(bool enabled) {
        slider_eps_cmd->setEnabled(enabled);
        line_edit_eps_cmd->setEnabled(enabled);
    }

    void setACCSliderEnabled(bool enabled) {
        slider_acc_cmd->setEnabled(enabled);
        line_edit_acc_cmd->setEnabled(enabled);
    }

    void setTargetSpeedEnabled(bool enabled) {
        slider_target_speed->setEnabled(enabled);
        line_edit_target_speed->setEnabled(enabled);
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<my_custom_msgs::msg::ControlMessage1>::SharedPtr publisher_;
    rclcpp::Publisher<my_custom_msgs::msg::ControlMessage2>::SharedPtr publisher2_;
    QTimer *message_timer_;
    bool eps_en_, controlsw_, scc_en_, turn_sig_L_, turn_sig_R_;
    int eps_cmd_, acc_cmd_, target_speed_;
    QPushButton *button_eps_en, *button_controlsw, *button_scc_en, *button_turn_sig_L, *button_turn_sig_R;
    QSlider *slider_eps_cmd, *slider_acc_cmd, *slider_target_speed;
    QLabel *label_eps_cmd, *label_acc_cmd, *label_target_speed;
    QLineEdit *line_edit_eps_cmd, *line_edit_acc_cmd, *line_edit_target_speed;
    double eps_min_, eps_max_, acc_min_, acc_max_;

    void setupPublisher() {
        publisher_ = node_->create_publisher<my_custom_msgs::msg::ControlMessage1>("KIAPI/Control1", 10);
        publisher2_ = node_->create_publisher<my_custom_msgs::msg::ControlMessage2>("KIAPI/Control2", 10);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("qt_ros2_node");
    QtRos2Node window(node);

    window.show();
    int result = app.exec();

    rclcpp::shutdown();
    return result;
}

#include "qt_ros2_node.moc"
