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

# ฟังก์ชันการจัดการข้อมูลเซ็นเซอร์ ToF
def tof_data_handler(sub_info):
    distance = sub_info
    global tof_distance
    tof_distance = distance[0]
    #print("tof1: {0}".format(distance[0]))

# ฟังก์ชันการจัดการข้อมูลเซ็นเซอร์
def sub_data_handler(sub_info):
    io_data, ad_data = sub_info
    global ad_data_value
    ad_data_value = ad_data
    print("io value: {0}, ad value: {1}".format(io_data, ad_data))
    

# ฟังก์ชันตรวจสอบเส้นทางข้างหน้า
def front_wall():
    if tof_distance is not None:
        return tof_distance < 250  # ถ้าต่ำกว่า 200 คือเจอกำแพง
    return False

# ฟังก์ชันตรวจสอบทางด้านซ้าย
def left_wall():
    if ad_data_value[3] is not None:
        # ใช้ sensor ด้านซ้าย 
        return ad_data_value[3] > 100  # Assuming left sensor is at index 3
    return False

# ฟังก์ชันตรวจสอบทางด้านขวา
def right_wall():
    if ad_data_value[2] is not None:
        # ใช้ sensor ด้านขวา 
        return ad_data_value[2] > 100  # Assuming right sensor is at index 2
    return False

# ฟังก์ชันการแก้ปัญหาเขาวงกต
def wall_following_solve():
    while True:
        time.sleep(1)
        if right_wall():
            if  front_wall() == False:
                move_forward()
            elif front_wall():
                turn_left()
        elif right_wall() == False:
            turn_right()
            move_forward()
        ep_gimbal.recenter().wait_for_completed()
        time.sleep(1)  # เพิ่ม delay เพื่อป้องกันการทำงานเร็วเกินไป

# เริ่มการสำรวจและแก้ปัญหาเขาวงกต
if __name__ == '__main__':
    ep_sensor = ep_robot.sensor_adaptor
    ep_gimbal = ep_robot.gimbal
    ep_gimbal.recenter().wait_for_completed()
    ep_sensor.sub_adapter(freq=5, callback=sub_data_handler)
    ep_tof = ep_robot.sensor
    ep_tof.sub_distance(freq=5, callback=tof_data_handler)
    try:
        wall_following_solve()
    except KeyboardInterrupt:
        print("Process interrupted")
    finally:
        ep_sensor.unsub_adapter()
        ep_tof.unsub_adapter()
        ep_robot.close()
