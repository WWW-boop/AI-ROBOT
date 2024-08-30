import robomaster
from robomaster import robot
import time

# ฟังก์ชันการเคลื่อนไหว
def move_forward():
    ep_chassis.move(x=0.6, y=0, z=0, xy_speed=1).wait_for_completed()

def turn_left():
    ep_chassis.move(x=0, y=0, z=90, z_speed=30).wait_for_completed()

def turn_right():
    ep_chassis.move(x=0, y=0, z=-90, z_speed=30).wait_for_completed()

# ตัวแปรเพื่อเก็บข้อมูลเซ็นเซอร์
sensor_data = {
    'io_value': None,
    'ad_value': None,
    'distance': None,
    'tof_status': False
}

# ฟังก์ชันการจัดการข้อมูลเซ็นเซอร์ ToF
def tof_data_handler(sub_info):
    distance = sub_info[0]
    sensor_data['distance'] = distance
    sensor_data['tof_status'] = 100 < distance < 200
    print(f"ToF Status: {sensor_data['tof_status']}, Distance: {distance} mm")
    time.sleep(1)

# ฟังก์ชันการจัดการข้อมูลเซ็นเซอร์
def sub_data_handler(sub_info):
    io_data, ad_data = sub_info
    sensor_data['io_value'] = io_data
    sensor_data['ad_value'] = ad_data
    print(f"io value: {io_data}, ad value: {ad_data}")

# ฟังก์ชันตรวจสอบเส้นทางข้างหน้า
def front_wall():
    if sensor_data['ad_value'] is not None:
        return sensor_data['ad_value'][0] < 100  # สมมุติว่าค่า ad_value ที่มากกว่า 100 หมายถึงมีสิ่งกีดขวาง
    return False

# ฟังก์ชันตรวจสอบทางด้านซ้าย
def left_wall():
    if sensor_data['ad_value'] is not None:
        return sensor_data['ad_value'][3] > 100  # ใช้ sensor ด้านซ้าย
    return False

# ฟังก์ชันตรวจสอบทางด้านขวา
def right_wall():
    if sensor_data['ad_value'] is not None:
        return sensor_data['ad_value'][2] > 100  # ใช้ sensor ด้านขวา
    return False

# ฟังก์ชันการแก้ปัญหาเขาวงกตแบบตามผนังด้านซ้าย
def left_wall_following_solve():
    while True:
        if not left_wall():
            print("No left wall, turning left.")
            turn_left()
        elif front_wall():
            print("Front wall detected, turning right.")
            turn_right()
        else:
            print("Following left wall, moving forward.")
            move_forward()

        if sensor_data['tof_status']:
            print("ToF sensor detected an object within range.")
            # Logic to handle ToF sensor detection can be added here, if necessary

        time.sleep(1)  # เพิ่ม delay เพื่อป้องกันการทำงานเร็วเกินไป

# เริ่มการสำรวจและแก้ปัญหาเขาวงกต
if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    time.sleep(1)

    ep_chassis = ep_robot.chassis
    ep_gimbal = ep_robot.gimbal
    ep_gimbal.recenter().wait_for_completed()

    ep_sensor = ep_robot.sensor_adaptor
    ep_sensor.sub_adapter(freq=5, callback=sub_data_handler)

    ep_tof = ep_robot.sensor
    ep_tof.sub_distance(freq=5, callback=tof_data_handler)

    try:
        left_wall_following_solve()
    except KeyboardInterrupt:
        print("Process interrupted")
    finally:
        ep_sensor.unsub_adapter()
        ep_tof.unsub_distance()
        ep_robot.close()
        print("Robot disconnected.")
