import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import math

from . import db

class SpatReceiverNode(Node):
    def __init__(self):
        super().__init__('spat_receiver_node')
        
        self.intersections = []
        self.current_node = None
        self.distance_threshold = 80.0 # meters
        
        self.prev_lat = None
        self.prev_lon = None
        self.current_heading = None
        
        # Initialize and cache intersections
        self.load_intersections()
        
        # Subscribe to GPS data
        self.subscription = self.create_subscription(
            NavSatFix,
            '/novatel/oem7/fix',
            self.gps_callback,
            10)
            
        # Setup Timer for DB Polling (e.g., 1 Hz)
        self.timer_period = 1.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.get_logger().info('SPaT Receiver Node initialized. Using GPS topic: /novatel/oem7/fix')

    def load_intersections(self):
        self.get_logger().info('Loading intersections from tod_info...')
        try:
            rows = db.select("SELECT id, x_coord, y_coord, int_nm FROM public.tod_info")
            for row in rows:
                node_id = row[0]
                x_coord = row[1]
                y_coord = row[2]
                name = row[3]
                
                if x_coord is not None and y_coord is not None:
                    lon = x_coord / 10000000.0
                    lat = y_coord / 10000000.0
                    
                    # PG intersections have been added to tod_info but their data is in spat_data
                    node_type = 'pg' if '자율주행' in name else 'tod'
                    
                    self.intersections.append({
                        'id': node_id,
                        'name': name,
                        'lat': lat,
                        'lon': lon,
                        'type': node_type
                    })
            self.get_logger().info(f'Loaded {len(self.intersections)} intersections from DB.')
        except Exception as e:
            self.get_logger().error(f'Failed to load intersections: {e}')

    def calculate_heading(self, lat1, lon1, lat2, lon2):
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
        initial_bearing = math.atan2(x, y)
        initial_bearing = math.degrees(initial_bearing)
        compass_bearing = (initial_bearing + 360) % 360
        return compass_bearing

    def get_target_direction_code(self, heading):
        if heading is None:
            return None
            
        # Entry direction code mapping based on vehicle heading (ref. kiapidb_spat_table_설명자료.pdf)
        # 10: North, 50: NE, 20: East, 60: SE, 30: South, 70: SW, 40: West, 80: NW
        # Note: DB direction_code represents entry approach direction (진입 방향).
        # Vehicle heading is the travel direction, so entry approach is the opposite side.
        if 337.5 <= heading or heading < 22.5: return 30     # Heading North -> Entry from South (30)
        elif 22.5 <= heading < 67.5: return 70              # Heading NE    -> Entry from SW (70)
        elif 67.5 <= heading < 112.5: return 40             # Heading East  -> Entry from West (40)
        elif 112.5 <= heading < 157.5: return 80            # Heading SE    -> Entry from NW (80)
        elif 157.5 <= heading < 202.5: return 10            # Heading South -> Entry from North (10)
        elif 202.5 <= heading < 247.5: return 50            # Heading SW    -> Entry from NE (50)
        elif 247.5 <= heading < 292.5: return 20            # Heading West  -> Entry from East (20)
        elif 292.5 <= heading < 337.5: return 60            # Heading NW    -> Entry from SE (60)
        return None

    def timer_callback(self):
        if self.current_node is None:
            return
            
        try:
            node_id = self.current_node['id']
            node_type = self.current_node['type']
            target_code = self.get_target_direction_code(self.current_heading)
            
            if node_type == 'pg':
                query = "SELECT * FROM public.spat_data where node_id = %s"
            else:
                query = "SELECT * FROM public.tod_spat_data where node_id = %s"
                
            rows_spat = db.select(query, (node_id,))
            
            if rows_spat:
                filtered_rows = []
                if target_code is not None:
                    for row in rows_spat:
                        if row[5] == target_code:  # direction_code is at index 5
                            filtered_rows.append(row)
                else:
                    # Heading not calculated yet (e.g. stopped since start)
                    filtered_rows = rows_spat
                
                if filtered_rows:
                    heading_str = f"{self.current_heading:.1f}도, 진입방향코드: {target_code}" if target_code else "방향 미산출"
                    self.get_logger().info(f"==== [{self.current_node['name']}] (진행방향: {heading_str}) ====")
                    for row in filtered_rows:
                        signal_state = row[2]
                        traffic_light = row[3]
                        remain_time_raw = row[4]
                        
                        # 보행자 신호(traffic_light == 3) 제외
                        if traffic_light == 3:
                            continue
                        
                        state_str = "알 수 없음"
                        if signal_state == 1: state_str = "\033[91m빨간불(Red)\033[0m"
                        elif signal_state == 2: state_str = "\033[93m노란불(Yellow)\033[0m"
                        elif signal_state == 3: state_str = "\033[92m초록불(Green)\033[0m"
                        elif signal_state == 6: state_str = "\033[92m초록 점멸(Blink)\033[0m"
                        
                        light_type = "기타"
                        if traffic_light == 1: light_type = "직진"
                        elif traffic_light == 2: light_type = "좌회전"
                        # elif traffic_light == 3: light_type = "보행자"
                        
                        if remain_time_raw == 255:
                            time_str = "점멸/계산불가"
                        else:
                            if node_type == 'pg':
                                time_str = f"{remain_time_raw}초"
                            else:
                                time_str = f"{remain_time_raw / 10.0:.1f}초"
                            
                        self.get_logger().info(f"  -> {light_type} 신호: {state_str} (남은시간: {time_str})")
                else:
                    self.get_logger().info(f"[{self.current_node['name']}] 현재 진행방향(코드 {target_code})에 해당하는 신호가 없습니다.")
            else:
                self.get_logger().debug(f"No SPaT data received for {self.current_node['name']}.")
                
        except Exception as e:
            self.get_logger().error(f'Database error: {e}')

    def gps_callback(self, msg):
        current_lat = msg.latitude
        current_lon = msg.longitude
        
        if current_lat == 0.0 or current_lon == 0.0:
            return
            
        # Update heading only if moved more than 0.5 meters
        if self.prev_lat is not None and self.prev_lon is not None:
            dist_moved = self.calculate_distance(self.prev_lat, self.prev_lon, current_lat, current_lon)
            if dist_moved > 0.5:
                self.current_heading = self.calculate_heading(self.prev_lat, self.prev_lon, current_lat, current_lon)
                self.prev_lat = current_lat
                self.prev_lon = current_lon
        else:
            self.prev_lat = current_lat
            self.prev_lon = current_lon
            
        closest_distance = float('inf')
        closest_node = None
        
        for intersection in self.intersections:
            distance = self.calculate_distance(current_lat, current_lon, intersection['lat'], intersection['lon'])
            
            # Check if intersection is behind vehicle (already passed)
            if self.current_heading is not None:
                bearing_to_int = self.calculate_heading(current_lat, current_lon, intersection['lat'], intersection['lon'])
                heading_diff = abs(self.current_heading - bearing_to_int)
                if heading_diff > 180.0:
                    heading_diff = 360.0 - heading_diff
                if heading_diff > 90.0:
                    continue

            if distance < closest_distance:
                closest_distance = distance
                closest_node = intersection
                
        if closest_node and closest_distance <= self.distance_threshold:
            if self.current_node is None or self.current_node['id'] != closest_node['id']:
                self.get_logger().info(f"Entered intersection: {closest_node['name']} (ID: {closest_node['id']}), Distance: {closest_distance:.2f}m")
            self.current_node = closest_node
        else:
            if self.current_node is not None:
                self.get_logger().info(f"Left the intersection area ({self.current_node['name']}).")
            self.current_node = None
        
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000 
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

def main(args=None):
    rclpy.init(args=args)
    node = SpatReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
