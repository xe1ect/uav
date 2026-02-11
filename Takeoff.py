from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
print("Connected to Aircraft!")

def upload_takeoff_mission(pitch=30, alt=46):
    print(f"Uploading Takeoff Mission: Pitch {pitch}, Alt {alt}m")
    
    # แจ้งว่าจะส่ง Mission จำนวน 2 ข้อความ (Waypoints)
    # Point 0: เป็นค่าปัจจุบัน (Home)
    # Point 1: คือคำสั่ง Takeoff
    master.mav.mission_count_send(master.target_system, master.target_component, 2)
    
    msg = master.recv_match(type=['MISSION_REQUEST'], blocking=True)
    master.mav.mission_item_send(
        master.target_system, master.target_component, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, 0, 0, 0)

    msg = master.recv_match(type=['MISSION_REQUEST'], blocking=True)
    master.mav.mission_item_send(
        master.target_system, master.target_component, 1,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        1, # เป็น Mission ปัจจุบัน
        1, # Auto continue
        pitch, # Param 1: Pitch Angle (องศาการเชิดหัว)
        0, 0, 0, 0, 0, alt) # Param 7: Target Altitude (ความสูงเป้าหมาย)

    # รอรับ Ack เพื่อยืนยันว่า Upload สำเร็จ
    ack = master.recv_match(type='MISSION_ACK', blocking=True)
    print(f"Mission Uploaded: {ack.type}")

def start_takeoff():
    # เปลี่ยนโหมดเป็น GUIDED เพื่อ Arm
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4) # 4 คือเลข ID ของโหมด GUIDED ใน ArduPlane

    # Arm มอเตอร์
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    
    print("Arming and waiting for motors...")
    master.motors_armed_wait()
    
    # เปลี่ยนเป็นโหมด AUTO เพื่อเริ่มวิ่งตาม Mission
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        10) # 10 คือเลข ID ของโหมด AUTO ใน ArduPlane
    print("Mode set to AUTO. Takeoff starting now!")

# --- ขั้นตอนการทำงาน ---
upload_takeoff_mission(pitch=30, alt=46) # ตั้งค่ามุมเชิด 12 องศา, ความสูง 30 เมตร
time.sleep(1)
start_takeoff()