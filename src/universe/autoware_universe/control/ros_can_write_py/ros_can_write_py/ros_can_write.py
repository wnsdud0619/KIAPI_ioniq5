import cantools
import can
import rclpy
from rclpy.node import Node
from my_custom_msgs.msg import ControlMessage1, ControlMessage2

#class can710():

class CanPublisher(Node):
    def __init__(self):
        super().__init__('can_publisher')

        # DBC 파일 로딩
        self.dbc = cantools.database.load_file('/home/kiapi/gui_ws/KIAPI.dbc')  # DBC 파일 경로
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')  # CAN 인터페이스 설정

        # 초기화할 변수들
        self.eps_cmd = 0  # 초기 eps_cmd 값
        self.acc_cmd = 1023  # 초기 acc_cmd 값
        self.eps_en = False
        self.controlsw = False
        self.eps_interval = 0
        self.scc_en = False
        self.aeb_act = False
        self.aeb_decel_value = 0
        self.turn_sig_hazard = False
        self.turn_sig_l = False
        self.turn_sig_r = False
        self.alive_cnt = 0
        self.accel_override_switch = False  # Accel_override_Switch 추가

        # ROS2 메시지를 구독하는 코드 추가
        self.create_subscription(
            ControlMessage1,
            'KIAPI/Control1',
            self.control_message1_callback,
            10
        )

        self.create_subscription(
            ControlMessage2,
            'KIAPI/Control2/cmd_conv',
            self.control_message2_callback,
            10
        )

        # 10ms마다 CAN 메시지를 보내는 주기 설정
        self.create_timer(0.01, self.timer_callback)  # 10ms 주기

    def control_message1_callback(self, msg: ControlMessage1):
        # ControlMessage1 메시지에서 데이터를 가져옵니다.
        self.eps_en = msg.eps_en
        self.controlsw = msg.controlsw
        self.eps_interval = msg.eps_interval
        #self.eps_interval = 200
        self.scc_en = msg.scc_en
        self.aeb_act = msg.aeb_act
        self.aeb_decel_value = msg.aeb_decel_value
        self.turn_sig_hazard = msg.turn_sig_hazard
        self.turn_sig_l = msg.turn_sig_l
        self.turn_sig_r = msg.turn_sig_r
        self.accel_override_switch = msg.accel_override_switch  # Accel_override_Switch 값 추가

    def control_message2_callback(self, msg: ControlMessage2):
        # ControlMessage2 메시지에서 데이터를 가져옵니다.
        self.eps_cmd = msg.kiapi_eps_cmd
        self.acc_cmd = msg.kiapi_acc_cmd

    def timer_callback(self):
        # Alive_Cnt 값을 0부터 255까지 1씩 증가시키고, 255가 되면 0으로 초기화
        self.alive_cnt = (self.alive_cnt + 1) % 256  # Alive_Cnt 값이 255에 도달하면 0으로 되돌림

        # DBC에서 해당 메시지 찾기
        message1 = self.dbc.get_message_by_name('CONTROL_MSG_1')  # DBC에서 메시지 이름을 사용
        if message1:
            # ControlMessage1 데이터를 DBC 신호에 매핑
            signals1 = {
                'EPS_En': self.eps_en,
                'ControlSW': self.controlsw,
                #'EPS_Interval': self.eps_interval * 1,
                'EPS_Interval': 12.0,
                'SCC_En': self.scc_en,
                'AEB_Act': self.aeb_act,
                'AEB_decel_value': self.aeb_decel_value * 1,
                'turn_sig_hazard': self.turn_sig_hazard,
                'turn_sig_L': self.turn_sig_l,
                'turn_sig_R': self.turn_sig_r,
                'Alive_Cnt': self.alive_cnt,
                'Accel_override_Switch': self.accel_override_switch  # Accel_override_Switch 값을 추가
            }

            # ControlMessage1에 대한 CAN 메시지 생성 및 전송
            can_message1 = message1.encode(signals1)
            self.bus.send(can.Message(arbitration_id=message1.frame_id, data=can_message1))
            #self.get_logger().info(f"Published CAN message for CONTROL_MSG_1 with ID: 0x{message1.frame_id:X}")
        else:
            self.get_logger().warn("Message 'CONTROL_MSG_1' not found in DBC")

        # DBC에서 CONTROL_MSG_2에 대한 메시지 처리
        message2 = self.dbc.get_message_by_name('CONTROL_MSG_2')  # DBC에서 메시지 이름을 사용
        if message2:
            # ControlMessage2 데이터를 DBC 신호에 매핑
            # acc_cmd_value = self.acc_cmd * 0.01 - 10.23
            # if acc_cmd_value < -3:
            #     acc_cmd_value = -3  # Minimum value as per the DBC file's signal specification
            # elif acc_cmd_value > 255:
            #     acc_cmd_value = 255  # Ensure it doesn't exceed the maximum value

            #print(self.acc_cmd)
            self.eps_cmd = min(self.eps_cmd, 500)
            self.eps_cmd = max(self.eps_cmd, -500)
            #print(self.eps_cmd)
            signals2 = {
            
                'EPS_Cmd': self.eps_cmd,
                'ACC_Cmd': (self.acc_cmd*0.01) - 10.23,
            }

            # ControlMessage2에 대한 CAN 메시지 생성 및 전송
            can_message2 = message2.encode(signals2)
            self.bus.send(can.Message(arbitration_id=message2.frame_id, data=can_message2))
            #self.get_logger().info(f"Published CAN message for CONTROL_MSG_2 with ID: 0x{message2.frame_id:X}")
        else:
            self.get_logger().warn("Message 'CONTROL_MSG_2' not found in DBC")

def main(args=None):
    rclpy.init(args=args)
    can_publisher = CanPublisher()
    rclpy.spin(can_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
