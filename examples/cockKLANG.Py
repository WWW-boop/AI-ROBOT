import robomaster
from robomaster import robot
import time
import matplotlib.pyplot as plt
import csv
from datetime import datetime

# เริ่มต้นหุ่นยนต์ RoboMaster
ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")


# ฟังก์ชันการเคลื่อนไหว
def move_forward():
    print("move_forward called")
    for i in range(2):
        ep_robot.chassis.move(x=0.3, y=0, z=0, xy_speed=10).wait_for_completed()
        ep_robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

def turn_left():
    print("turn_left called")
    ep_robot.chassis.move(x=0, y=0, z=90, z_speed=100).wait_for_completed()
    ep_robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

def turn_right():
    print("turn_right called")
    ep_robot.chassis.move(x=0, y=0, z=-91, z_speed=100).wait_for_completed()
    ep_robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    
def turn_around():
    print("turn_around called")
    ep_robot.chassis.move(x=0, y=0, z=180, z_speed=100).wait_for_completed()
    ep_robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

# ฟังก์ชันการจัดการข้อมูลเซ็นเซอร์ ToF
def tof_data_handler(sub_info):
    print("tof_data_handler called")
    distance = sub_info
    global tof_distance
    tof_distance = distance[0]
    print("ToF: {0}".format(distance[0]))


# ฟังก์ชันการจัดการข้อมูลเซ็นเซอร์
def sub_data_handler(sub_info):
    print("sub_data_handler called")
    io_data, ad_data = sub_info
    global ad_data_value
    global dis_ssL
    global dis_ssR
    ad_data_value = ad_data
    ssR = ad_data_value[3]
    ssL = ad_data_value[2]
    vaR, vaL = convert_to_V(ssR, ssL)
    dis_ssR = convert_to_cm(vaR)
    dis_ssL = convert_to_cm(vaL)
    if ssL[2] > 150:
        dis_ssL = 40

    print(f"Distance From Right Wall: {dis_ssR} cm")
    print(f"Distance From Left Wall: {dis_ssL} cm")

def convert_to_V(ssR,ssL):
    print("convert_to_V called")
    # Assuming the sensor value (ssR) needs to be converted to centimeters.
    # ssR, ssL = filter_signal(ssR,ssL)
    ad_data_vo_ssr = (ssR * 3) / 1023
    ad_data_vo_ssl = (ssL * 3) / 1023
    # freak_handler(ad_data_vo_ssr)
    return ad_data_vo_ssr , ad_data_vo_ssl

def convert_to_cm(voltage):
    print("convert_to_cm called")
    if voltage > 1.4:
        cm = (voltage - 4.2) / -0.31
    elif 1.4 >= voltage >= 0.6:
        cm = (voltage - 2.03) / -0.07
    else:
        cm = (voltage - 0.95) / -0.016
    
    return cm

# ฟังก์ชันตรวจสอบเส้นทางข้างหน้า
def front_wall():
    print("front_wall called")
    if tof_distance is not None:
        return tof_distance < 400  # ถ้าต่ำกว่า 200 คือเจอกำแพง
    return False

# ฟังก์ชันตรวจสอบทางด้านซ้าย
def left_wall():
    print("left_wall called")
    if g_dis_l < 360 :
        return True 
    return False

# ฟังก์ชันตรวจสอบทางด้านขวา
def right_wall():
    print("right_wall called")
    if dis_ssR < 30 :
        return True
    return False

def stick_left_wall():
    print("stick_left_wall called")
    global err_dis_l
    if dis_ssL < 40:
        err_dis_l = (dis_ssL-23)/100
        if abs(err_dis_l) >= 0.01:
            return True
    return False

def stick_front_wall():
    print("stick_front_wall called")
    global err_dis_f
    err_dis_f = (tof_distance-200)/1000
    if abs(err_dis_f) >= 0.01:
        return True
        # print(f"stick_front_wall returns {result}")
    return False

def gimbal_wall_check():
    global g_dis_l
    global g_dis_f
    ep_gimbal.moveto(pitch=0, yaw=-90, pitch_speed=100, yaw_speed=200).wait_for_completed()
    g_dis_l = tof_distance
    ep_gimbal.moveto(pitch=0, yaw=0, pitch_speed=100, yaw_speed=200).wait_for_completed()
    g_dis_f = tof_distance
        

# ฟังก์ชันการแก้ปัญหาเขาวงกต
def wall_following_solve():
    print("wall_following_solve called")
    time.sleep(1)
    while True:
        gimbal_wall_check()
        if left_wall(): #ถ้าเจอกำแพงด้านขวา
            if stick_left_wall() == True:
                ep_robot.chassis.move(x=0, y=-err_dis_l, z=0, xy_speed=5).wait_for_completed()
                ep_robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
            if  front_wall() == False: #ถ้าไม่พบกำแพงข้างหน้า
                move_forward() # เคลื่อนที่ไปข้างหน้า 1 กระเบื้อง
            elif front_wall():
                if stick_front_wall() == True:
                    ep_robot.chassis.move(x=err_dis_f, y=0, z=0, xy_speed=5).wait_for_completed()
                    ep_robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0) # ถ้าพบกำแพงทั้งข้างหน้าและขวา
                if right_wall() == True:
                    turn_around()
                else:
                    turn_right()
        elif left_wall() == False: 
            if front_wall() == True:   
                if stick_front_wall() == True:
                        ep_robot.chassis.move(x=err_dis_f, y=0, z=0, xy_speed=5).wait_for_completed()
                        ep_robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
            turn_left() # ให้หันขวาและตรงไป 
            ep_gimbal.moveto(pitch=0, yaw=0, pitch_speed=100, yaw_speed=200).wait_for_completed()
            if front_wall() == True:
                if stick_front_wall() == True:
                        ep_robot.chassis.move(x=err_dis_f, y=0, z=0, xy_speed=5).wait_for_completed()
                        ep_robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
            move_forward()
            gimbal_wall_check()
            if left_wall() == False:
                turn_left()
                ep_gimbal.moveto(pitch=0, yaw=0, pitch_speed=100, yaw_speed=200).wait_for_completed()
                move_forward()
        ep_gimbal.moveto(pitch=0, yaw=0, pitch_speed=100, yaw_speed=200).wait_for_completed() #ขยับgimbal มาตรงกลางทุกครั้งที่ทำตามคำสั่งข้างต้น


def sub_position_handler(position_info):
    print("sub_position_handler called")
    x, y, z = position_info
    elapsed_time = time.time() - start_time
    print("chassis position: x:{0}, y:{1}, time:{2}".format(x, y, elapsed_time))
    writer.writerow([x, y, elapsed_time])
    file.flush()  # Ensure data is written to the file immediately
    x_data.append(x)
    y_data.append(y)
    global current_y
    current_y = y

# เริ่มการสำรวจและแก้ปัญหาเขาวงกต
if __name__ == '__main__':
    print("__main__ called")
    x_data = []
    y_data = []
    start_time = time.time()
    with open('position_data.csv', mode='a', newline='') as file:  # Open in append mode
        writer = csv.writer(file)
        writer.writerow(["x", "y", "time"])

        ep_chassis = ep_robot.chassis
        ep_sensor = ep_robot.sensor_adaptor
        ep_gimbal = ep_robot.gimbal
        ep_gimbal.recenter().wait_for_completed()
        ep_sensor.sub_adapter(freq=20, callback=sub_data_handler)
        ep_tof = ep_robot.sensor
        ep_tof.sub_distance(freq=20, callback=tof_data_handler)
        ep_chassis.sub_position(freq=20, callback=sub_position_handler)
        try:
            wall_following_solve()
        except KeyboardInterrupt:
            print("Process interrupted")
        finally:
            ep_sensor.unsub_adapter()
            ep_tof.unsub_adapter()
            ep_chassis.unsub_position()
            ep_robot.close()