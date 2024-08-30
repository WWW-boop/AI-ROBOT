import robomaster
from robomaster import robot
import time
import csv

ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")

def move_forward():
    print("move_forward called")
    for _ in range(2):
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

def tof_data_handler(sub_info):
    print("tof_data_handler called")
    global tof_distance
    tof_distance = sub_info[0]
    print("ToF: {0}".format(sub_info[0]))

def sub_data_handler(sub_info):
    print("sub_data_handler called")
    io_data, ad_data = sub_info
    global dis_ssL, dis_ssR
    ssR = ad_data[3]
    ssL = ad_data[2]
    vaR, vaL = convert_to_V(ssR, ssL)
    dis_ssR = convert_to_cm(vaR)
    dis_ssL = convert_to_cm(vaL)
    if ssL[2] > 150:
        dis_ssL = 40

    print(f"Distance From Right Wall: {dis_ssR} cm")
    print(f"Distance From Left Wall: {dis_ssL} cm")

def convert_to_V(ssR,ssL):
    print("convert_to_V called")
    ad_data_vo_ssr = (ssR * 3) / 1023
    ad_data_vo_ssl = (ssL * 3) / 1023
    return ad_data_vo_ssr, ad_data_vo_ssl

def convert_to_cm(voltage):
    print("convert_to_cm called")
    if voltage > 1.4:
        cm = (voltage - 4.2) / -0.31
    elif 1.4 >= voltage >= 0.6:
        cm = (voltage - 2.03) / -0.07
    else:
        cm = (voltage - 0.95) / -0.016
    return cm

def front_wall():
    print("front_wall called")
    if tof_distance is not None:
        return tof_distance < 400
    return False

def left_wall():
    print("left_wall called")
    if dis_ssL < 30 :
        return True 
    return False

def right_wall():
    print("right_wall called")
    if g_dis_r < 400 :
        return True
    return False

def stick_right_wall():
    print("stick_right_wall called")
    global err_dis_r
    if dis_ssR < 40:
        err_dis_r = (dis_ssR-23)/100
        if abs(err_dis_r) >= 0.01:
            return True
    return False

def stick_front_wall():
    print("stick_front_wall called")
    global err_dis_f
    err_dis_f = (tof_distance-200)/1000
    if abs(err_dis_f) >= 0.01:
        return True
    return False

def gimbal_wall_check():
    global g_dis_r, g_dis_f
    ep_gimbal.moveto(pitch=0, yaw=90, pitch_speed=100, yaw_speed=200).wait_for_completed()
    g_dis_r = tof_distance
    # default_gimbal()
    g_dis_f = tof_distance

def adjust_r_position():
    ep_robot.chassis.move(x=0, y=err_dis_r, z=0, xy_speed=5).wait_for_completed()
    ep_robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

def adjust_f_position():
    ep_robot.chassis.move(x=err_dis_f, y=0, z=0, xy_speed=5).wait_for_completed()
    ep_robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

# def default_gimbal():
    ep_gimbal.moveto(pitch=0, yaw=0, pitch_speed=100, yaw_speed=200).wait_for_completed()

def wall_following_solve():
    print("wall_following_solve called")
    time.sleep(1)
    while True:
        gimbal_wall_check()
        if right_wall(): #ถ้าเจอกำแพงด้านขวา
            if stick_right_wall() == True:
                adjust_r_position()
            if  front_wall() == False: #ถ้าไม่พบกำแพงข้างหน้า
                move_forward() # เคลื่อนที่ไปข้างหน้า 1 กระเบื้อง
            elif front_wall():
                if stick_front_wall() == True:
                    adjust_f_position()
                if left_wall() == True:
                    turn_around()
                else:
                    turn_left() #ให้หันซ้าย
        elif right_wall() == False: #ถ้าไม่พบกำแพงขวา
            if front_wall() == True:   
                if stick_front_wall() == True:
                        adjust_f_position()
            turn_right() # ให้หันขวาและตรงไป 
            # default_gimbal()
            move_forward()
            gimbal_wall_check()
            if front_wall() == True:
                if stick_front_wall() == True:
                        adjust_f_position()
            if right_wall() == False:
                turn_right()
                # default_gimbal()
                move_forward()
            if front_wall() == True:
                if stick_front_wall() == True:
                        adjust_f_position()
        # default_gimbal()


def sub_position_handler(position_info):
    print("sub_position_handler called")
    x, y, z = position_info
    elapsed_time = time.time() - start_time
    print("chassis position: x:{0}, y:{1}, time:{2}".format(x, y, elapsed_time))
    writer.writerow([x, y, elapsed_time])
    file.flush()
    x_data.append(x)
    y_data.append(y)
    global current_y
    current_y = y

if __name__ == '__main__':
    print("__main__ called")
    x_data = []
    y_data = []
    start_time = time.time()
    with open('position_data.csv', mode='a', newline='') as file:
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