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

def turn_back():
    ep_robot.chassis.move(x=0, y=0, z=180, xy_speed=1).wait_for_completed()
    #ep_robot.chassis.move(x=0, y=0, z=180, z_speed=30).wait_for_completed()
# ตัวแปรเพื่อเก็บข้อมูลเซ็นเซอร์
sensor_data = {
    'io_value': None,
    'ad_value': None,
    'distance': None
}

# ฟังก์ชันการจัดการข้อมูลเซ็นเซอร์ ToF
def tof_data_handler(sub_info):
    distance = sub_info
    sensor_data['distance'] = distance[0]
    #print("tof1: {0}".format(distance[0]))

# ฟังก์ชันการจัดการข้อมูลเซ็นเซอร์
def sub_data_handler(sub_info):
    io_data, ad_data = sub_info
    sensor_data['io_value'] = io_data
    sensor_data['ad_value'] = ad_data
    print("io value: {0}, ad value: {1}".format(io_data, ad_data))
    

# ฟังก์ชันตรวจสอบเส้นทางข้างหน้า
def front_wall():
    if sensor_data['distance'] is not None:
        return sensor_data['distance'] < 200  # ถ้าต่ำกว่า 200 คือเจอกำแพง
    return False

# ฟังก์ชันตรวจสอบทางด้านซ้าย
def left_wall():
    if sensor_data['ad_value'] is not None:
        # ใช้ sensor ด้านซ้าย 
        return sensor_data['ad_value'][3] > 100  # Assuming left sensor is at index 3
    return False

# ฟังก์ชันตรวจสอบทางด้านขวา
def right_wall():
    if sensor_data['ad_value'] is not None:
        # ใช้ sensor ด้านขวา 
        return sensor_data['ad_value'][2] > 100  # Assuming right sensor is at index 2
    return False

# ฟังก์ชันการแก้ปัญหาเขาวงกต
def wall_following_solve():
    time.sleep(1)
    while True:
        ep_gimbal.recenter().wait_for_completed()
        # 0 0 0 ใช้ตอนเจอสามแยก
        if front_wall() == False and right_wall() == False and left_wall() == False:
            turn_right()
            move_forward()
        # 0 0 1 ใช้ตอนเจอทางแยกขวา
        elif front_wall() == False and right_wall() == False and left_wall() == True:
            turn_right()
            move_forward()
        # 0 1 0 ใช้ตอนเจอทางแยกซ้าย
        elif front_wall() == False and right_wall() == True and left_wall() == False:
            move_forward()
        # 0 1 1 ใช้ตอนทางตรงกําแพงซ้ายขวา
        elif front_wall() == False and right_wall() == True and left_wall() == True:
            move_forward()
        # 1 0 0 ใช้ตอนทางแยกซ้ายขวา
        elif front_wall() == True and right_wall() == False and left_wall() == False:
            turn_right()
            move_forward()
        # 1 0 1 ใช้ตอนเลี้ยวขวา
        elif front_wall() == True and right_wall() == False and left_wall() == True:
            turn_right()
            move_forward()    
        # 1 1 0 ใช้ตอนเลี้ยวซ้าย
        elif front_wall() == True and right_wall() == True and left_wall() == False:
            turn_left()
            move_forward()  
        # 1 1 1 ใช้ตอนเจอทางตัน
        elif front_wall() == True and right_wall() == True and left_wall() == True:
            turn_back()
            move_forward()  
        else :
            turn_right()
            
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
