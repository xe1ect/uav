from pymavlink import mavutil
import time
import math

class GEO:
    R_EARTH = 6378137
    
    @staticmethod
    def get_offset(lat1, lon1, lat2, lon2):
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        lat1_rad = math.radians(lat1)
        
        north_m = dlat * GEO.R_EARTH
        east_m = dlon * GEO.R_EARTH * math.cos(lat1_rad)
        return north_m, east_m
    
    @staticmethod
    def apply(ref_lat,ref_lon, north_m, east_m):
        dlat = north_m / GEO.R_EARTH
        dlon = east_m / (GEO.R_EARTH * math.cos(math.radians(ref_lat)))
        
        new_lat = ref_lat + math.degrees(dlat)
        new_lon = ref_lon + math.degrees(dlon)
        return new_lat, new_lon
    
    @staticmethod
    def tranform(templete_wps, current_lat, current_lon):
        new_wps = []
        
        if len(templete_wps[0]) ==3:
            ref_lat, ref_lon, _ = templete_wps[0]
        else:
            ref_lat, ref_lon = templete_wps[0]
        for item in templete_wps:
            if len(item) ==3:
                lat, lon, alt= item
            else:
                lat, lon = item
                alt= None
            
            n_off, e_off = GEO.get_offset(ref_lat, ref_lon, lat, lon)
            new_lat,new_lon = GEO.apply(current_lat, current_lon, n_off, e_off)
            if alt is not None:
                new_wps.append((new_lat, new_lon,alt))
            else:
                new_wps.append((new_lat, new_lon))
            
        return new_wps
    
class UAV:
    def __init__(self, connection_string,baud=57600):
        self.conn_str= connection_string
        self.baud = baud
        self.master = None
        self.system_id = 0
        self.component_id = 0
        
    def connect(self):
        self.master = mavutil.mavlink_connection(self.conn_str, baud=self.baud)
        self.master.wait_heartbeat()
        self.system_id = self.master.target_system
        self.component_id = self.master.target_component
        print(f"Connected to UAV with System ID: {self.system_id}, Component ID: {self.component_id}")

    def get_connection_info(self):
        while True:
            msg= self.master.recv_match(type="GPS_RAW_INT", blocking=True, timeout=1)
            if msg:
                if msg.fix_type >= 3:
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    return lat, lon
                
                if int(time.time()) %2 ==0:
                    print(".",end="",flush=True)
                time.sleep(0.5)
    
    def set_param_verify(self, param_id, param_value, param_type):
        
        # แปลง Input param_id ให้เป็น String และ Bytes เพื่อใช้แยกกัน
        if isinstance(param_id, bytes):
            param_id_bytes = param_id
            param_id_str = param_id.decode()
        else:
            param_id_bytes = param_id.encode('utf-8')
            param_id_str = param_id

        print(f"Setting {param_id_str} to {param_value}...")
        
        for i in range(5):
            self.master.mav.param_set_send(
                self.system_id, self.component_id,
                param_id_bytes, param_value, param_type
            )
            
            time.sleep(0.2)
        return True
    
    def get_altitude(self):
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
        if msg:
            return msg.relative_alt / 1000.0 # mm to meters
        return 0.0

    def set_mode(self, mode):
        if mode not in self.master.mode_mapping():
            return False
        
        mode_id = self.master.mode_mapping()[mode]
        start_time = time.time()
        while time.time() - start_time < 5:
            self.master.mav.set_mode_send(
                self.system_id,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )

            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg:
                if msg.custom_mode == mode_id:
                    print(f"Mode set to {mode}")
                    return True
            time.sleep(0.5)
        return False
            
    def arm_check(self):
        self.set_param_verify(b'ARMING_CHECK', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    
    def get_forward_point(self, lat, lon, distance=100):
        new_lat, new_lon = GEO.apply(lat, lon, distance, 0)
        return new_lat, new_lon
    
    def upload_mission(self, waypoints, takeoff_alt=40,altitude=40, add_landing=True, land_lat=None, land_lon=None):
        if not self.master: raise Exception("UAV not connected")
        
        print("Clearing old mission")
        self.master.mav.mission_clear_all_send(self.system_id, self.component_id)
        self.master.recv_match(type=['MISSION_ACK'], blocking=True, timeout=3)
        mission_items = []
        seq=0
        
        mission_items.append(mavutil.mavlink.MAVLink_mission_item_int_message(
            self.system_id, self.component_id, seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 0, 0, 0, 0, 0,
            int(land_lat * 1e7), int(land_lon * 1e7),
            0
        ))
        seq += 1   

        hand_off=15
        
        climb_lat, climb_lon = self.get_forward_point(land_lat, land_lon, distance=50)
        
        mission_items.append(mavutil.mavlink.MAVLink_mission_item_int_message(
            self.system_id, self.component_id, seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 1,
            10,
            0, 0, 0, 
            int(climb_lat * 1e7), int(climb_lon * 1e7),
            float(hand_off)
        ))
        seq+=1
        
        if len(waypoints[0]) ==3:
            first_wp_lat, first_wp_lon, _ = waypoints[0]
        else:
            first_wp_lat, first_wp_lon = waypoints[0]
        
        for data in waypoints:
            if len(data) == 3:
                lat, lon, alt= data
            else:
                lat, lon = data
                alt = altitude
            msg = mavutil.mavlink.MAVLink_mission_item_int_message(
                self.system_id, self.component_id, seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 1,
                0, 0, 0, 0,
                int(lat * 1e7), int(lon * 1e7),
                float(alt)
            )
            mission_items.append(msg)
            seq +=1
            
        mission_items.append(mavutil.mavlink.MAVLink_mission_item_int_message(
            self.system_id, self.component_id, seq,
            mavutil.mavlink.MAV_FRAME_MISSION,
            mavutil.mavlink.MAV_CMD_DO_LAND_START,
            0, 1, 0, 0, 0, 0, 0, 0, 0
        ))
        seq +=1
        
        approach_lat, approach_lon = GEO.apply(home_lat,home_lon, 300, 0)

        mission_items.append(mavutil.mavlink.MAVLink_mission_item_int_message(
            self.system_id, self.component_id, seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT,
            0, 1,
            0, 0, 0, 0,
            int(approach_lat * 1e7), int(approach_lon * 1e7),
            30
        ))
        seq +=1
        
        mission_items.append(mavutil.mavlink.MAVLink_mission_item_int_message(
            self.system_id, self.component_id, seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 1,
            0, 0, 0, 0,
            int(land_lat * 1e7), int(land_lon * 1e7),
            0
        ))
        seq +=1
        
        for i,item in enumerate(mission_items):
            item.seq = i
        
        total_items = len(mission_items)

        self.master.mav.mission_count_send(self.system_id, self.component_id, total_items)
        
        start_time = time.time()
        while True:
            msg = self.master.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'], blocking=True, timeout=1)
            
            if not msg:
                # Retry sending count if stuck
                if time.time() - start_time > 2.0:
                    self.master.mav.mission_count_send(self.system_id, self.component_id, total_items)
                continue

            if msg.get_type() in ['MISSION_REQUEST', 'MISSION_REQUEST_INT']:
                req_seq = msg.seq
                if req_seq < total_items:
                    # print(f"Sending item {req_seq}")
                    self.master.mav.send(mission_items[req_seq])
                else:
                    print(f"Error: Request unknown seq {req_seq}")

            elif msg.get_type() == 'MISSION_ACK':
                if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    print("✅ Mission Upload Success!")
                    break
                else:
                    print(f"❌ Mission Upload Failed code: {msg.type}")
                    break
                    
            if time.time() - start_time > 20:
                raise Exception("Upload Timeout")
        
        return total_items
        
    def start_mission(self, target_alt=40,total_items=0):

        self.master.mav.mission_set_current_send(self.system_id, self.component_id, 1)
        time.sleep(1)
        
        if not self.set_mode("AUTO"):
            print("Cant switch to auto")
            return
        
        print("Arming motors...")
        self.master.mav.command_long_send(
            self.system_id, self.component_id,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0 
        )
        
        self.master.motors_armed_wait()
        print("✅ Motors ARMED!")

        while True:
            msg = self.master.recv_match(type='MISSION_CURRENT', blocking=True, timeout=2)
            if msg:
                if msg.seq >= total_items: 
                    print("Mission sequence finished.")
            
            hb = self.master.recv_match(type='HEARTBEAT', blocking=False)
            if hb:
                armed = hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                if not armed and msg.seq > 1:
                    print("🏁 Disarmed (Landed). End of script.")
                    break
            time.sleep(1)
        
if __name__ == "__main__":
    TEMPLETE_WPS = [
        (52.779867, -0.712086, 40), (52.780494, -0.712729, 40),
        (52.781224, -0.709372, 40), (52.782885, -0.710350, 40),
        (52.782032, -0.710870, 40), (52.782783, -0.708481, 40),
        (52.783655, -0.710516, 40), (52.781427, -0.712204, 40),
        (52.782054, -0.707315, 20), (52.781395, -0.706479, 15),
        (52.780940, -0.708120, 6), (52.780940, -0.708370,  6),
        (52.780920, -0.708620,  6), (52.780900, -0.708870,  6)

    ]
    
    try:
        drone = UAV("udp:127.0.0.1:14550")
        drone.connect()
        
        home_lat, home_lon = drone.get_connection_info()
        
        drone.arm_check()
        
        local_mission = GEO.tranform(TEMPLETE_WPS, home_lat, home_lon)
        
        total= drone.upload_mission(local_mission,takeoff_alt=40,altitude=40, land_lat=float(home_lat), land_lon=float(home_lon))
        
        time.sleep(2)
        drone.start_mission(total_items=total)
        
    except Exception as e:
        print(f"Error: {e}")
        