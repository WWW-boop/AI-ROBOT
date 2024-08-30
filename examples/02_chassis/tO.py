import robomaster
from robomaster import robot
import time
import matplotlib.pyplot as plt
import csv

# เริ่มต้นหุ่นยนต์ RoboMaster
ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")

x_data = []
y_data = []

def sub_position_handler(position_info):
    x, y, z = position_info
    print("Chassis position: x:{0}, y:{1}, z:{2}".format(x, y, z))
    writer.writerow([x, y, z])
    x_data.append(x)
    y_data.append(y)

# ฟังก์ชันการเคลื่อนไหว
def move_forward():
    ep_robot.chassis.move(x=0.3, y=0, z=0, xy_speed=10).wait_for_completed()
    ep_robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

def turn_left():
    ep_robot.chassis.move(x=0, y=0, z=90, z_speed=30).wait_for_completed()
    ep_robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

def turn_right():
    ep_robot.chassis.move(x=0, y=0, z=-90, z_speed=30).wait_for_completed()
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
    return tof_distance is not None and tof_distance < 350

# ฟังก์ชันตรวจสอบทางด้านซ้าย
def left_wall():
    return dis_ssL < 25

# ฟังก์ชันตรวจสอบทางด้านขวา
def right_wall():
    return dis_ssR < 30

def stick_right_wall():
    err_dis_r = (dis_ssR - 23) / 100
    return abs(err_dis_r) >= 0.1

def stick_front_wall():
    err_dis_f = (tof_distance - 220) / 1000
    return abs(err_dis_f) >= 0.1

# ฟังก์ชันการแก้ปัญหาเขาวงกต
def wall_following_solve():
    try:
        while True:
            time.sleep(0.1)
            if right_wall():  # ถ้าเจอกำแพงด้านขวา
                if stick_right_wall():
                    ep_robot.chassis.move(x=0, y=(dis_ssR - 23) / 100, z=0, xy_speed=1).wait_for_completed()
                if not front_wall():  # ถ้าไม่พบกำแพงข้างหน้า
                    move_forward()  # เคลื่อนที่ไปข้างหน้า
                else:
                    if stick_front_wall():
                        ep_robot.chassis.move(x=(tof_distance - 220) / 1000, y=0, z=0, xy_speed=1).wait_for_completed()
                    turn_left()  # ให้หันซ้าย
            else:  # ถ้าไม่พบกำแพงขวา
                if front_wall():
                    if stick_front_wall():
                        ep_robot.chassis.move(x=(tof_distance - 220) / 1000, y=0, z=0, xy_speed=1).wait_for_completed()
                turn_right()  # ให้หันขวา
                if not right_wall():
                    move_forward()
                    turn_right()
                    move_forward()
    except KeyboardInterrupt:
        print("Process interrupted")

# เริ่มการสำรวจและแก้ปัญหาเขาวงกต
if __name__ == '__main__':
    with open('position_data.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["x", "y", "z"])

        ep_chassis = ep_robot.chassis
        ep_sensor = ep_robot.sensor_adaptor
        ep_gimbal = ep_robot.gimbal
        ep_gimbal.recenter().wait_for_completed()
        ep_sensor.sub_adapter(freq=5, callback=sub_data_handler)
        ep_tof = ep_robot.sensor
        ep_tof.sub_distance(freq=5, callback=tof_data_handler)
        ep_chassis.sub_position(freq=5, callback=sub_position_handler)

        try:
            wall_following_solve()
        except KeyboardInterrupt:
            plt.figure(figsize=(10, 6))
            plt.plot(x_data, y_data, marker='o')
            plt.title("Robot Chassis Position")
            plt.xlabel("X position")
            plt.ylabel("Y position")
            plt.grid(True)
            plt.axis('equal')
            plt.show()
        finally:
            ep_sensor.unsub_adapter()
            ep_tof.unsub_adapter()
            ep_chassis.unsub_position()
            ep_robot.close()
