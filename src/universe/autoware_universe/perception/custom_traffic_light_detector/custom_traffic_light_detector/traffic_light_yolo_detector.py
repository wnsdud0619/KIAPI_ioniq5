#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
from ultralytics import YOLO
import message_filters
import lanelet2

from tier4_perception_msgs.msg import TrafficLightRoiArray
from autoware_perception_msgs.msg import (
    TrafficLightGroupArray,
    TrafficLightGroup,
    TrafficLightElement
)

class TrafficLightYoloDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_yolo_detector')

        # 1. 파라미터 설정
        self.declare_parameter('model_path', '/home/kiapi/Documents/yolo10_ws/src/yolov10/model/best.pt')
        self.declare_parameter('osm_map_path', '/home/kiapi/KIAPI_ioniq5/map_data/lanelet2_map.osm')
        
        model_path = self.get_parameter('model_path').value
        osm_map_path = self.get_parameter('osm_map_path').value

        # 2. YOLO 모델 로드
        self.get_logger().info(f"Loading YOLO model from {model_path}...")
        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        # 3. Lanelet2 맵 로드 및 매핑
        self.physical_to_logical_id = {}
        self.load_lanelet_map(osm_map_path)

        # 4. Subscriber & Sync
        self.image_sub = message_filters.Subscriber(self, Image, "/sensing/camera/camera/image_color")
        self.roi_sub = message_filters.Subscriber(self, TrafficLightRoiArray, "/perception/traffic_light_recognition/camera/detection/rough/rois")
        
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.roi_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)

        # 5. Publisher
        self.image_pub = self.create_publisher(Image, "/yolov10_image/debug", 10)
        self.publisher_ = self.create_publisher(
            TrafficLightGroupArray,
            '/perception/traffic_light_recognition/traffic_signals',
            10
        )
        
        self.get_logger().info("Traffic Light YOLO Detector (Highest Confidence Global Mode) Initialized.")

    def load_lanelet_map(self, map_path):
        self.get_logger().info(f"Loading Lanelet2 Map from {map_path}...")
        try:
            projector = lanelet2.projection.MercatorProjector(lanelet2.io.Origin(0.0, 0.0))
            map_data = lanelet2.io.load(map_path, projector)
            for reg_elem in map_data.regulatoryElementLayer:
                if isinstance(reg_elem, lanelet2.core.TrafficLight):
                    logical_id = reg_elem.id
                    for light_linestring in reg_elem.trafficLights:
                        physical_id = light_linestring.id
                        if physical_id not in self.physical_to_logical_id:
                            self.physical_to_logical_id[physical_id] = []
                        self.physical_to_logical_id[physical_id].append(logical_id)
            self.get_logger().info(f"Map Loaded successfully. Found mapping for {len(self.physical_to_logical_id)} physical traffic lights.")
        except Exception as e:
            self.get_logger().error(f"Failed to load map: {e}")

    def sync_callback(self, img_msg, roi_msg):
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        results = self.model.track(source=cv_img, tracker='botsort.yaml', verbose=False)
        
        group_array_msg = TrafficLightGroupArray()
        group_array_msg.stamp = img_msg.header.stamp
        
        # 1. YOLO 결과 중 Confidence가 가장 높은 단 1개의 객체 찾기
        best_yolo_box = None
        highest_conf = -1.0

        if len(results) > 0 and len(results[0].boxes) > 0:
            for box in results[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                cls = int(box.cls[0])

                # 디버깅: 인식된 모든 YOLO 박스는 화면에 얇은 선으로 일단 표시
                color = self.get_color_for_class(cls)
                cv2.rectangle(cv_img, (x1, y1), (x2, y2), color, 1)

                if conf > highest_conf:
                    highest_conf = conf
                    best_yolo_box = {'bbox': (x1, y1, x2, y2), 'conf': conf, 'cls': cls}

        published_logical_ids = set() # 중복 ID 발행 방지용

        # 2. 가장 높은 Confidence 결과를 바탕으로 전체 신호등 요소(Elements) 생성
        if best_yolo_box is not None:
            # 1등으로 뽑힌 객체 시각화 (두꺼운 박스와 BEST 글씨 추가)
            bx1, by1, bx2, by2 = best_yolo_box['bbox']
            b_color = self.get_color_for_class(best_yolo_box['cls'])
            cv2.rectangle(cv_img, (bx1, by1), (bx2, by2), b_color, 4)
            label = f"BEST ({highest_conf:.2f})"
            cv2.putText(cv_img, label, (bx1, by1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, b_color, 2)

            # 이 신호등 클래스로 일괄 적용할 배열 생성
            elements_to_apply = self.create_elements_from_class(best_yolo_box['cls'], best_yolo_box['conf'])
        else:
            # YOLO가 화면에서 신호등을 아예 하나도 못 찾은 경우 (Unknown 상태 부여)
            elements_to_apply = self.create_elements_from_class(-1, 0.0)

        # 3. 맵에서 요구하는 모든 신호등 ID에 똑같은 결과를 집어넣기
        for rough_roi in roi_msg.rois:
            physical_id = rough_roi.traffic_light_id

            if physical_id in self.physical_to_logical_id:
                logical_ids = self.physical_to_logical_id[physical_id]
                for logical_id in logical_ids:
                    if logical_id not in published_logical_ids:
                        group = TrafficLightGroup()
                        group.traffic_light_group_id = logical_id
                        # 모든 논리적 ID에 가장 높은 Confidence를 가진 동일한 상태 부여
                        for el in elements_to_apply:
                            group.elements.append(el)
                        group_array_msg.traffic_light_groups.append(group)
                        published_logical_ids.add(logical_id)
            else:
                # 만약 맵 매핑 관계가 딕셔너리에 없는 예외 케이스라도 일단 발행
                if physical_id not in published_logical_ids:
                    group = TrafficLightGroup()
                    group.traffic_light_group_id = physical_id
                    for el in elements_to_apply:
                        group.elements.append(el)
                    group_array_msg.traffic_light_groups.append(group)
                    published_logical_ids.add(physical_id)

        # 최종 Publish
        self.publisher_.publish(group_array_msg)

        # 디버깅 이미지 Publish
        debug_img_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
        debug_img_msg.header = img_msg.header
        self.image_pub.publish(debug_img_msg)

    def get_color_for_class(self, class_id):
        colors = {
            0: (0, 0, 0), 1: (255, 0, 0), 2: (0, 255, 255), 3: (0, 0, 255), 
            4: (255, 0, 0), 5: (0, 255, 255), 6: (0, 0, 255), 7: (255, 0, 0), 
            8: (255, 0, 0), 9: (0, 0, 255), 10: (0, 0, 255), 11: (255, 0, 0), 
            12: (0, 0, 255)
        }
        return colors.get(class_id, (255, 255, 255))

    def create_element(self, color, shape, status, confidence):
        element = TrafficLightElement()
        element.color = color
        element.shape = shape
        element.status = status
        element.confidence = float(confidence)
        return element

    def create_elements_from_class(self, _tl_result, conf):
        elements = []
        if _tl_result == 4: # Green
            elements.append(self.create_element(3, 1, 2, conf))
        elif _tl_result == 5: # Yellow
            elements.append(self.create_element(2, 1, 2, conf))
        elif _tl_result == 6: # Red
            elements.append(self.create_element(1, 1, 2, conf))
        elif _tl_result == 7: # Green_Left
            elements.append(self.create_element(3, 1, 2, conf))
            elements.append(self.create_element(3, 2, 2, conf))
        elif _tl_result == 9: # Red_Left
            elements.append(self.create_element(1, 1, 2, conf))
            elements.append(self.create_element(3, 2, 2, conf))
        else: # Unknown
            elements.append(self.create_element(0, 0, 0, conf))
        return elements

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightYoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down YOLO Traffic Light Node")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()