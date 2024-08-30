import robomaster
from robomaster import robot
import time

# เริ่มต้นหุ่นยนต์ RoboMaster
ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")

# ฟังก์ชันการเคลื่อนไหว
def move_forward():
    ep_robot.chassis.move(x=0.6, y=0, z=0, xy_speed=1).wait_for_completed()

def turn_left():
    ep_robot.chassis.move(x=0, y=0, z=90, z_speed=30).wait_for_completed()

def turn_right():
    ep_robot.chassis.move(x=0, y=0, z=-90, z_speed=30).wait_for_completed()

# ตัวแปรเพื่อเก็บข้อมูลเซ็นเซอร์
sensor_data = {
    'io_value': None,
    'ad_value': None
}

# ฟังก์ชันการจัดการข้อมูลเซ็นเซอร์
def sub_data_handler(sub_info):
    io_data, ad_data = sub_info
    sensor_data['io_value'] = io_data
    sensor_data['ad_value'] = ad_data
    print("io value: {0}, ad value: {1}".format(io_data, ad_data))
    

# ฟังก์ชันตรวจสอบเส้นทางข้างหน้า
def is_path_clear():
    # สมมุติว่าค่า ad_value ที่มากกว่า 100 หมายถึงมีสิ่งกีดขวาง
    if sensor_data['ad_value'] is not None:
        return sensor_data['ad_value'] < 100 
    return False

# ฟังก์ชันการแก้ปัญหาเขาวงกต
def wall_following_solve():
    while True:
        if is_path_clear():
            move_forward()
        else:
            turn_left()  # ถ้าทางเดินข้างหน้ามีสิ่งกีดขวาง ให้เลี้ยวซ้าย
            if is_path_clear():
                move_forward()
            else:
                turn_right()
                turn_right()  # ถ้ายังไม่ว่าง ให้เลี้ยวขวา 2 ครั้งเพื่อกลับหลัง

# เริ่มการสำรวจและแก้ปัญหาเขาวงกต
if __name__ == '__main__':
    ep_sensor = ep_robot.sensor_adaptor
    ep_sensor.sub_adapter(freq=5, callback=sub_data_handler)
    try:
        wall_following_solve()
    except KeyboardInterrupt:
        print("Process interrupted")
    finally:
        ep_sensor.unsub_adapter()
        ep_robot.close()
