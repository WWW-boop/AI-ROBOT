import robomaster
from robomaster import robot
import time

# เริ่มต้นหุ่นยนต์ RoboMaster
ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")

# ฟังก์ชันการเคลื่อนไหว
def move_forward():
    ep_robot.chassis.move(x=0.1, y=0, z=0, xy_speed=10).wait_for_completed()
    ep_robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

def turn_left():
    ep_robot.chassis.move(x=0, y=0, z=90, z_speed=30).wait_for_completed()

def turn_right():
    ep_robot.chassis.move(x=0, y=0, z=-90, z_speed=30).wait_for_completed()

# ฟังก์ชันการจัดการข้อมูลเซ็นเซอร์ ToF
tof_distance = None  # กำหนดค่าเริ่มต้นให้กับตัวแปร global
def tof_data_handler(sub_info):
    global tof_distance
    tof_distance = sub_info[0]
    print("tof1: {0}".format(tof_distance))

# ฟังก์ชันการจัดการข้อมูลเซ็นเซอร์
dis_ssL, dis_ssR = None, None  # กำหนดค่าเริ่มต้นให้กับตัวแปร global
def sub_data_handler(sub_info):
    global dis_ssL, dis_ssR
    io_data, ad_data = sub_info

    ssR = ad_data[2]
    ssL = ad_data[3]

    vaR, vaL = convert_to_V(ssR, ssL)
    dis_ssR = convert_to_cm(vaR)
    dis_ssL = convert_to_cm(vaL)

    print(f"Distance ssR: {dis_ssR} cm")
    print(f"Distance ssL: {dis_ssL} cm")

def convert_to_V(ssR, ssL):
    ad_data_vo_ssr = (ssR * 3) / 1023
    ad_data_vo_ssl = (ssL * 3) / 1023
    return ad_data_vo_ssr, ad_data_vo_ssl

def convert_to_cm(voltage):
    if voltage > 1.4:
        cm = (voltage - 4.2) / -0.31
    elif 1.4 >= voltage >= 0.6:
        cm = (voltage - 2.03) / -0.07
    else:
        cm = (voltage - 0.95) / -0.016
    return cm

# ฟังก์ชันตรวจสอบเส้นทางข้างหน้า
def front_wall():
    return tof_distance is not None and tof_distance < 230

# ฟังก์ชันตรวจสอบทางด้านซ้าย
def left_wall():
    return dis_ssL is not None and dis_ssL < 25

# ฟังก์ชันตรวจสอบทางด้านขวา
def right_wall():
    return dis_ssR is not None and dis_ssR < 40

# ฟังก์ชันติดตามกำแพงขวา
def stick_right_wall():
    if dis_ssR is not None:
        err_dis_r = (dis_ssR - 20) / 100
        if abs(err_dis_r) >= 0.1:
            ep_robot.chassis.move(x=0, y=err_dis_r, z=0, xy_speed=1).wait_for_completed()
        return True
    return False

# ฟังก์ชันการแก้ปัญหาเขาวงกต
def wall_following_solve():
    while True:
        time.sleep(0.5)
        if right_wall():
            stick_right_wall()
            if not front_wall():
                move_forward()
            else:
                turn_left()
        else:
            for i in range(3):
                move_forward()
            turn_right()
            ep_gimbal.recenter().wait_for_completed()
            for i in range(7):
                move_forward()
            if not right_wall():
                turn_right()
                ep_gimbal.recenter().wait_for_completed()
                for i in range(7):
                    move_forward()

        ep_gimbal.recenter().wait_for_completed()

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
