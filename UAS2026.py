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
    def tranform(templete_wps, current_lat, current_lon, heading_deg=0, manual_ref=None):
        new_wps = []
        
        def unpack_wp(wp):
            if len(wp) == 2 and isinstance(wp[0], (list, tuple)):
                return float(wp[0][0]), float(wp[0][1]), float(wp[1])
            elif len(wp) >= 3:
                return float(wp[0]), float(wp[1]), float(wp[2])
            else:
                return float(wp[0]), float(wp[1]), None
            
        if manual_ref:
             ref_lat, ref_lon = manual_ref[0], manual_ref[1]
        else:

             ref_lat, ref_lon, _ = unpack_wp(templete_wps[0])
            
        for item in templete_wps:
            lat, lon, alt = unpack_wp(item)
            
            n_off, e_off = GEO.get_offset(ref_lat, ref_lon, lat, lon)
            #n_rot, e_rot = GEO.rotate_point(n_off, e_off, heading_deg)  # No rotation applied
            new_lat, new_lon = GEO.apply(current_lat, current_lon, n_off,e_off)
            
            if alt is not None:
                new_wps.append((new_lat, new_lon, alt))
            else:
                new_wps.append((new_lat, new_lon))
            
        return new_wps
    
    @staticmethod
    def get_point(lat, lon, bearing_deg, distance_m):
        rad = math.radians(bearing_deg)

        north_m= distance_m * math.cos(rad)
        east_m= distance_m * math.sin(rad)
        
        return GEO.apply(lat, lon, north_m, east_m)
    
    @staticmethod
    def get_bearing(lat1, lon1, lat2, lon2):
        north_m, east_m = GEO.get_offset(lat1, lon1, lat2, lon2)
        bearing = math.degrees(math.atan2(east_m, north_m))
        if bearing < 0:
            bearing += 360
        return bearing
    
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

    def get_heading(self):
        for _ in range(10):
            msg = self.master.recv_match(type='VFR_HUD', blocking=True, timeout=1)
            if msg:
                return msg.heading
            time.sleep(0.1)
        return 0
    
    """def force_alt(self, target_alt):
        self.master.mav.command_long_send(
            self.system_id, self.component_id,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE,
            0,
            float(target_alt), 0, 0, 0, 0, 0, 0
        )"""
    
    def get_altitude(self):
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
        if msg:
            return msg.relative_alt / 1000.0 # mm to meters
        return 0.0

    def get_distance(lat1, lon1, lat2, lon2):
        north,east = GEO.get_offset(lat1, lon1, lat2, lon2)
        return math.sqrt(north**2 + east**2)
    
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
    
    
    def get_forward_point(self, lat, lon, heading, distance=100):
        new_lat, new_lon = GEO.get_point(lat, lon,heading, distance)
        return new_lat, new_lon
    
    def upload_mission(self, loop_wp, drop_wp,loop_alt=45,drop_alt=6,altitude=40, land_lat=None, land_lon=None,descent_dist=250,target_bearing=247.0):
        if not self.master: raise Exception("UAV not connected")
        
        
        current_heading= self.get_heading()
        #target_bearing= GEO.get_bearing(land_lat, land_lon, loop_wp[0][0], loop_wp[0][1])
        #print(target_bearing)
        
        print("Clearing old mission")
        self.master.mav.mission_clear_all_send(self.system_id, self.component_id)
        self.master.recv_match(type=['MISSION_ACK'], blocking=True, timeout=3)
        mission_items = []
        seq=0
        
        home_lat, home_lon = loop_wp[0][0], loop_wp[0][1]
        
        mission_items.append(mavutil.mavlink.MAVLink_mission_item_int_message(
            self.system_id, self.component_id, seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 0, 0, 0, 0, 0,
            int(land_lat * 1e7), int(land_lon * 1e7),
            0
        ))
        
        hand_off=30
        climb_lat, climb_lon = self.get_forward_point(land_lat, land_lon, target_bearing, distance=100)
        
        mission_items.append(mavutil.mavlink.MAVLink_mission_item_int_message(
            self.system_id, self.component_id, seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 1,
            30,
            0, 0, 0, 
            int(climb_lat * 1e7), int(climb_lon * 1e7),
            float(hand_off)
        ))
        seq = 2
        
        """far_dist = 130
        far_lat, far_lon = self.get_forward_point(land_lat, land_lon, current_heading, distance=far_dist)
        
        mission_items.append(mavutil.mavlink.MAVLink_mission_item_int_message(
            self.system_id, self.component_id, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1, 30, 0, 0, 0,
            int(far_lat * 1e7), int(far_lon * 1e7),
            30
        ))"""
        
        last_loop_lat = 0
        last_loop_lon = 0
        
        #for i in range(0,2):
        for i, wp in enumerate(loop_wp[1:]):
            lat, lon = wp[0], wp[1]
            last_loop_lat = lat 
            last_loop_lon = lon
            
            mission_items.append(mavutil.mavlink.MAVLink_mission_item_int_message(
                self.system_id, self.component_id, seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 0, 0, 0, 0, 0, int(lat*1e7), int(lon*1e7), float(loop_alt)
            ))
            seq += 1
        
        #drop_lat, drop_lon = drop_wp[0][0], drop_wp[0][1]
        
        """mission_items.append(mavutil.mavlink.MAVLink_mission_item_int_message(
            self.system_id, self.component_id, seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT,
            0, 1, 0, 40, 0, 1, int(drop_wp[0][0]), int(drop_wp[0][1]), float(drop_alt)
        ))
        seq+=1"""

        """first_drop_lat = drop_wp[0][0]
        first_drop_lon = drop_wp[0][1]
        
        dive_ratio = 0.2
        
        d_lat = first_drop_lat - last_loop_lat
        d_lon = first_drop_lon - last_loop_lon
        
        dive_lat = last_loop_lat + (d_lat * dive_ratio)
        dive_lon = last_loop_lon + (d_lon * dive_ratio)
        
        mission_items.append(mavutil.mavlink.MAVLink_mission_item_int_message(
            self.system_id, self.component_id, seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 0, 0, 0, 0, 0, 
            int(dive_lat * 1e7), int(dive_lon * 1e7), 
            float(drop_alt) # <<< บังคับลงต่ำตรงนี้
        ))
        seq += 1"""
        
        for i, wp in enumerate(drop_wp):
            lat, lon = wp[0], wp[1]
            """if i==0:
                mission_items.append(mavutil.mavlink.MAVLink_mission_item_int_message(
                    self.system_id, self.component_id, seq,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT,
                    0, 1, 0, 40, 0, 1, int(lat*1e7), int(lon*1e7), float(drop_alt)
                ))
                seq+=1"""
            
            mission_items.append(mavutil.mavlink.MAVLink_mission_item_int_message(
                self.system_id, self.component_id, seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 0, 0, 0, 0, 0, int(lat*1e7), int(lon*1e7), float(drop_alt)
            ))
            seq += 1
        
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
    (0.0, 0.0, 45.0),
    (52.7802127, -0.7094389, 45.0),
    (52.7801219, -0.7106915, 45.0),
    (52.7799807, -0.711696, 45.0),
    (52.7799588, -0.7118154, 45.0),
    (52.7799571, -0.7119414, 45.0),
    (52.7799912, -0.7120474, 45.0),
    (52.780063, -0.7120507, 45.0),
    (52.7801291, -0.7120071, 45.0),
    (52.7811893, -0.7126951, 45.0),
    (52.7813548, -0.712852, 45.0),
    (52.7813994, -0.7129459, 45.0),
    (52.7814684, -0.7129285, 45.0),
    (52.7814943, -0.7128178, 45.0),
    (52.7814453, -0.7127253, 45.0),
    (52.7821692, -0.7076392, 45.0),
    (52.7821652, -0.7075144, 45.0),
    (52.7821778, -0.7073971, 45.0),
    (52.7822203, -0.7072985, 45.0),
    (52.7822901, -0.7073287, 45.0),
    (52.7823124, -0.7074427, 45.0),
    (52.7837133, -0.7088388, 45.0),
    (52.7837749, -0.708873, 45.0),
    (52.783418, -0.7088656, 45.0),
    (52.7839108, -0.7088904, 45.0),
    (52.783957, -0.7089789, 45.0),
    (52.7839124, -0.7090701, 45.0),
    (52.7838422, -0.7090996, 45.0),
    (52.7828076, -0.7108471, 45.0),
    (52.7827857, -0.7109617, 45.0),
    (52.7827744, -0.711069, 45.0),
    (52.7827484, -0.7111401, 45.0),
    (52.782705, -0.7111736, 45.0),
    (52.7826559, -0.7111508, 45.0),
    (52.7826235, -0.7110885, 45.0),
    (52.7826231, -0.7110033, 45.0),
    (52.7826458, -0.7109322, 45.0),
    (52.7826805, -0.7108786, 45.0),
    (52.7832047, -0.7071027, 45.0),
    (52.7831893, -0.70698, 45.0),
    (52.7831848, -0.70686, 45.0),
    (52.7831978, -0.7067406, 45.0),
    (52.7832679, -0.7067044, 45.0),
    (52.7833215, -0.7067855, 45.0),
    (52.7833401, -0.7068989, 45.0),
    (52.7847292, -0.7095811, 45.0),
    (52.7847758, -0.709675, 45.0),
    (52.7848476, -0.7096555, 45.0),
    (52.7848938, -0.7097487, 45.0),
    (52.7848679, -0.7098607, 45.0),
    (52.7847969, -0.7098768, 45.0),
    (52.7822536, -0.711987, 45.0),
    (52.7820589, -0.712207, 45.0),
    (52.7820289, -0.7123089, 45.0),
    (52.7819567, -0.7123256, 45.0),
    (52.7819153, -0.7122264, 45.0),
    (52.7819458, -0.7121178, 45.0),
    (52.7819879, -0.7120199, 45.0),
    (52.7830185, -0.7070464, 45.0),
    (52.7830627, -0.7069498, 45.0),
    (52.783124, -0.7068841, 45.0),
    (52.7831479, -0.7067695, 45.0),
    (52.7830984, -0.7066796, 45.0),
    (52.7830433, -0.7065991, 45.0),
    (52.7818156, -0.7036266, 45.0),
    (52.7818099, -0.7035019, 45.0),
    (52.7817904, -0.7033879, 45.0),
    (52.7817509, -0.7032829, 45.0),
    (52.7816781, -0.70329, 45.0),
    (52.7816464, -0.7034019, 45.0),
    (52.7816541, -0.7035226, 45.0)
]
    DROP_WPS = [
    (52.7810466, -0.7068801, 6),
    (52.7808827, -0.7075024, 6)
]
    
    try:
        drone = UAV("udp:127.0.0.1:14550")
        drone.connect()
        
        time.sleep(2)
        
        home_lat, home_lon = drone.get_connection_info()
        current_heading = drone.get_heading()
        #drone.arm_check()
        
        local_mission = GEO.tranform(TEMPLETE_WPS, home_lat, home_lon, heading_deg=current_heading)
        
        loop_ref_lat = TEMPLETE_WPS[0][0]
        loop_ref_lon = TEMPLETE_WPS[0][1]
        
        drop_mission = GEO.tranform(DROP_WPS, home_lat, home_lon, heading_deg=current_heading, manual_ref=(loop_ref_lat, loop_ref_lon))
        
        total= drone.upload_mission(local_mission,drop_mission, land_lat=float(home_lat), land_lon=float(home_lon))
        
        time.sleep(2)
        drone.start_mission(total_items=total)
        
        """while True:
            drone.master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, 
                                            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            msg = drone.master.recv_match(type="MISSION_CURRENT", blocking=True, timeout=1)
            if msg:
                current_wp = msg.seq
                
                if current_wp >= 12:
                    curr_pos = drone.master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
                    if curr_pos:
                        curr_lat = curr_pos.lat / 1e7
                        curr_lon = curr_pos.lon / 1e7

                        target_lat, target_lon = drop_mission[-1][0], drop_mission[-1][1]
                        dist = GEO.get_distance(curr_lat, curr_lon, target_lat, target_lon)

                        if dist < 800:
                            drone.force_alt(6)
                            break
                            
                        
            time.sleep(0.5)"""
    except Exception as e:
        print(f"Error: {e}")
        