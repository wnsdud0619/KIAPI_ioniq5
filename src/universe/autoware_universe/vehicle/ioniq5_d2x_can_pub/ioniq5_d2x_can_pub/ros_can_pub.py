import rclpy
from rclpy.node import Node
from std_msgs.msg import Header  
import can
import cantools
import threading
import queue
import asyncio
from autoware_vehicle_msgs.msg import *
from rclpy.clock import Clock
from rclpy.executors import MultiThreadedExecutor
import math
import os
from ament_index_python.packages import get_package_share_directory

from vehicle_msgs.msg import DeviceD2XVehicleInfoTransmit #D2X

# 기어 변환 딕셔너리 (P:0, R:7, N:6, D:5)
Gear_DISP_dict = { 
    0 : 22,
    7 : 20,
    6 : 1,
    5 : 2,
}

# #D2X: 기어 변환 딕셔너리 (P:0, R:7, N:6, D:5)
D2X_Gear_DISP_dict = { 
    0 : 2,
    7 : 3,
    6 : 4,
    5 : 5,
}

# D2X: 지시등 (left:1, right:4, harzard:5)
D2X_Signal_DISP_dict = { 
    1 : 2,
    4 : 3,
    5 : 4,
}


class CanReceiver(Node):
    def __init__(self):
        super().__init__('can_receiver_node')
        
        self.d2x_msg = DeviceD2XVehicleInfoTransmit() #D2X

        # ROS 2 퍼블리셔 생성
        self.pub_GearReport = self.create_publisher(GearReport, '/vehicle/status/gear_status', 10) 
        self.pub_SteeringReport = self.create_publisher(SteeringReport, '/vehicle/status/steering_status', 10) 
        # self.pub_VelocityReport = self.create_publisher(VelocityReport, '/vehicle/status/velocity_status', 10) 
        self.pub_HazardLightsReport = self.create_publisher(HazardLightsReport, '/vehicle/status/hazard_lights_status', 10) 
        self.pub_TurnIndicatorsReport = self.create_publisher(TurnIndicatorsReport, '/vehicle/status/turn_indicators_status', 10) 
        self.pub_vehicle_status = self.create_publisher(DeviceD2XVehicleInfoTransmit, '/vehicle_info/vehicle_status/can', 10) # D2X

        # CAN 인터페이스 설정 (ThreadSafeBus 사용)
        self.bus = can.ThreadSafeBus(interface='socketcan', channel='can0')

        # DBC 파일 로드 및 메시지 캐싱 (속도 향상)
        pkg_share = get_package_share_directory('ioniq5_d2x_can_pub')
        dbc_path = os.path.join(pkg_share, 'KIAPI.dbc')
        self.dbc = cantools.database.load_file(dbc_path)
        self.dbc_messages = {msg.frame_id: msg for msg in self.dbc.messages}

        # CAN 메시지 수신 큐 (멀티스레드, 최대 크기 제한)
        self.msg_queue = queue.Queue(maxsize=1000)

        # CAN 데이터 수신을 비동기 스레드에서 실행
        self.can_thread = threading.Thread(target=self.can_receive_thread, daemon=True)
        self.can_thread.start()

        # 데이터 수신 및 퍼블리시 (5ms 주기)
        self.create_timer(0.002, self.timer_callback)  # 2ms 주기

    def can_receive_thread(self):
        """CAN 메시지를 별도 스레드에서 수신하여 큐에 저장"""
        while rclpy.ok():
            try:
                message = self.bus.recv(timeout=0.0001)  # 타임아웃 최소화
                if message:
                    if not self.msg_queue.full():  # 큐가 가득 찬 경우 데이터 삭제 방지
                        self.msg_queue.put_nowait(message)
            except queue.Full:
                pass  # 큐가 가득 차면 버퍼 오버플로우 방지
            
    def deg_to_rad(self, degree):
        return degree * (math.pi / 180)

    def autoware_can_message(self, message, decoded_signals, header):
        arb_id = message.arbitration_id

        if arb_id == 0x710:
            SteeringReport_msg = SteeringReport()
            SteeringReport_msg.steering_tire_angle = self.deg_to_rad(decoded_signals.get('StrAng', 0))
            SteeringReport_msg.steering_tire_angle = -SteeringReport_msg.steering_tire_angle / 13.5
            SteeringReport_msg.stamp = header.stamp
            self.pub_SteeringReport.publish(SteeringReport_msg)

        elif arb_id == 0x711:
            GearReport_msg = GearReport()
            gear_disp = decoded_signals.get('Gear_Disp', 0)
            gear_value = getattr(gear_disp, "value", gear_disp)
            GearReport_msg.report = Gear_DISP_dict.get(gear_value, 0) 
            
            #GearReport_msg.report = Gear_DISP_dict.get(gear_disp, 0)
            GearReport_msg.stamp = header.stamp
            self.pub_GearReport.publish(GearReport_msg)

        #elif arb_id == 0x713:
            #VelocityReport_msg = VelocityReport()
            #VelocityReport_msg.longitudinal_velocity = decoded_signals.get('Long_Accel', 0)
            #VelocityReport_msg.lateral_velocity = decoded_signals.get('Lat_Accel', 0)
            #VelocityReport_msg.heading_rate = decoded_signals.get('Yaw_Rate', 0)
            #VelocityReport_msg.header = header
            #self.pub_VelocityReport.publish(VelocityReport_msg)

        elif arb_id == 0x715:
            HazardLightsReport_msg = HazardLightsReport()
            TurnIndicatorsReport_msg = TurnIndicatorsReport()

            #msg에 can값 넣어서 변하도록 해야함 현재는 기본값만 넣음
            HazardLightsReport_msg.report = 2  # DISABLE = 1, ENABLE = 2
            TurnIndicatorsReport_msg.report = 2 # No command = 0, DISABLE = 1, ENABLE_LEFT = 2, ENABLE_RIGHT = 3
            HazardLightsReport_msg.stamp = header.stamp
            TurnIndicatorsReport_msg.stamp = header.stamp
            self.pub_HazardLightsReport.publish(HazardLightsReport_msg)
            self.pub_TurnIndicatorsReport.publish(TurnIndicatorsReport_msg)
    
    # D2X 추가
    def d2x_can_message(self, message, decoded_signals):
        arb_id = message.arbitration_id

        msg = self.d2x_msg

        motion = msg.vehicle_info.vehicle_status.motion
        inner = msg.vehicle_info.vehicle_status.inner_parts

        if message.arbitration_id == 0x710:
            inner.steer.angle = int(decoded_signals.get('StrAng', 0) * 10) # 10배 스케일링하면 [-4800 ~ 4800]
            # inner.steer.torque = decoded_signals.get('Str_Tq_2', 0)

        elif message.arbitration_id == 0x711:
            GearReport_msg = GearReport()
            motion.velocity.kph  = decoded_signals.get('Vehicle_speed') # 시속
            motion.velocity.mps = int(motion.velocity.kph * (1000.0 / 3600.0)) # 초속
            gear_disp = decoded_signals.get('Gear_Disp', 0)
            gear_value = getattr(gear_disp, "value", gear_disp)
            inner.transmission_gear = D2X_Gear_DISP_dict.get(gear_value, 0)                
                     
        #elif message.arbitration_id == 0x715:
         #   inner.pedal.accel = decoded_signals.get('accel_position', 0) # 안됨, None
          #  inner.pedal.brake = decoded_signals.get('brake_position', 0) # 안됨, None
            # inner.signal_light = decoded_signals['turn_signal_left'] # 바뀌긴 하나 문제 있음 11111->00000->11111....
            # inner.signal_light = decoded_signals['trun_signal_right'] # 바뀌긴 하나 문제 있음 11111->00000->11111....
            # inner.signal_light = decoded_signals['trun_signal_right'] # 안됨, None은 아니고 그냥 0으로만 나옴
                        
        self.pub_vehicle_status.publish(msg)   

    def timer_callback(self):
        """메인 스레드에서 CAN 데이터를 가져와 퍼블리시"""
        while not self.msg_queue.empty():
            try:
                message = self.msg_queue.get_nowait()
            except queue.Empty:
                return  # 큐가 비어 있으면 처리 안 함
                
            decoded_signals = self.decode_message(message)
            if not decoded_signals:
                continue  # 디코딩 실패 시 스킵

            # 공통 헤더 설정
            header = Header()
            header.frame_id = 'base_link'
            header.stamp = Clock().now().to_msg()

            # autoware CAN 메세지 
            self.autoware_can_message(message, decoded_signals, header)

            # d2x CAN 메세지
            self.d2x_can_message(message, decoded_signals)

    def decode_message(self, message):
        """캐싱된 DBC 메시지를 사용하여 디코딩"""
        try:
            can_message = self.dbc_messages.get(message.arbitration_id)
            if can_message:
                return can_message.decode(message.data)
        except Exception as e:
            self.get_logger().error(f"Failed to decode message: {e}")
        return {}

def main(args=None):
    rclpy.init(args=args)

    # 다중 스레드 실행을 위한 Executor 설정
    executor = MultiThreadedExecutor()
    can_receiver_node = CanReceiver()
    executor.add_node(can_receiver_node)

    # 병렬 실행 시작
    try:
        executor.spin()
    finally:
        can_receiver_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
