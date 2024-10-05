import robomaster
from robomaster import robot
import time

# เริ่มต้นหุ่นยนต์ RoboMaster
ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")

# ฟังก์ชันการเคลื่อนไหว
def move_forward():
    ep_robot.chassis.move(x=0.3, y=0, z=0, xy_speed=10).wait_for_completed()
    ep_robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

def turn_left():
    ep_robot.chassis.move(x=0, y=0, z=90, z_speed=80).wait_for_completed()
    ep_robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

def turn_right():
    ep_robot.chassis.move(x=0, y=0, z=-90, z_speed=80).wait_for_completed()
    ep_robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

# ฟังก์ชันการจัดการข้อมูลเซ็นเซอร์ ToF
def tof_data_handler(sub_info):
    distance = sub_info
    global tof_distance
    tof_distance = distance[0]
    print("tof1: {0}".format(distance[0]))

# ฟังก์ชันการจัดการข้อมูลเซ็นเซอร์
def sub_data_handler(sub_info):
    io_data, ad_data = sub_info
    global dis_ssL
    global dis_ssR
    ssR = ad_data[3]
    ssL = ad_data[2]
    dis_ssR = convert_to_cm(convert_to_V(ssR))
    dis_ssL = convert_to_cm(convert_to_V(ssL))
    print(f"Distance ssR: {dis_ssR}  cm")
    print(f"Distance ssL: {dis_ssL}  cm")

def convert_to_V(ss):
    return (ss * 3) / 1023

def convert_to_cm(voltage):
    if voltage > 1.4:
        return (voltage - 4.2) / -0.31
    elif 1.4 >= voltage >= 0.6:
        return (voltage - 2.03) / -0.07
    else:
        return (voltage - 0.95) / -0.016

# ฟังก์ชันตรวจสอบ壁
def check_walls():
    front = tof_distance < 350  # เช็คกำแพงข้างหน้า
    left = dis_ssL < 25  # เช็คกำแพงด้านซ้าย
    right = dis_ssR < 25  # เช็คกำแพงด้านขว
    return front, left, right

# ฟังก์ชัน DFS สำรวจ
def dfs(explored):
    front, left, right = check_walls()

    if (0, 0) not in explored:
        explored.append((0, 0))

    if front:
        turn_left()  # หันซ้าย
        dfs(explored)
        turn_right()  # หันขวาเพื่อกลับ
    elif right:
        move_forward()  # ไปข้างหน้า
        dfs(explored)
    elif left:
        turn_left()  # หันซ้าย
        move_forward()  # ไปข้างหน้า
        dfs(explored)
        turn_right()  # หันขวาเพื่อกลับ

# เริ่มการสำรวจ
if __name__ == '__main__':
    ep_sensor = ep_robot.sensor_adaptor
    ep_gimbal = ep_robot.gimbal
    ep_gimbal.recenter().wait_for_completed()
    ep_sensor.sub_adapter(freq=5, callback=sub_data_handler)
    ep_tof = ep_robot.sensor
    ep_tof.sub_distance(freq=5, callback=tof_data_handler)
    
    explored = []  # รายการที่บันทึกตำแหน่งที่สำรวจแล้ว
    try:
        dfs(explored)
    except KeyboardInterrupt:
        print("Process interrupted")
    finally:
        ep_sensor.unsub_adapter()
        ep_tof.unsub_adapter()
        ep_robot.close()
