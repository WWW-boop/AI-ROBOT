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

def filter_ad_data(ad_data):
    filtered_data = []
    smoothing_factor = 0.1  # ค่าอัลฟาสำหรับการกรอง
    previous_value = 0  # กำหนดค่าเริ่มต้นของ previous_value

    for reading in ad_data:
        current_value = smoothing_factor * previous_value + (1 - smoothing_factor) * reading
        filtered_data.append(current_value)
        previous_value = current_value

    return filtered_data

# ฟังก์ชันการจัดการข้อมูลเซ็นเซอร์
def sub_data_handler(sub_info): #sensor sharp เอาไว้เช็คกำแพงว่าห่างจาก กำแพงทำไร แล้วปรับค่าให้อยู่ตรงกลางตลอดของกำแพง
    io_data, ad_data = sub_info
    global dis_ssL, dis_ssR

    # กรองค่า ADC ก่อน
    smoothed_values = filter_ad_data(ad_data)
    
    # แปลงค่า ADC เป็นแรงดันไฟฟ้า
    ssR = smoothed_values[1] 
    ssL = smoothed_values[0] 
    vaR, vaL = convert_to_V(ssR, ssL)

    # แปลงค่าแรงดันไฟฟ้าเป็นระยะทาง
    dis_ssR = convert_to_cm(vaR) / 2
    dis_ssL = convert_to_cm(vaL) / 2 

    print(f"Distance ssR: {dis_ssR} cm")
    print(f"Distance ssL: {dis_ssL} cm")
    
    # ตรวจสอบกำแพง
    wall_left = check_wall_left(io_data)
    wall_right = check_wall_right(io_data)

    print(f'Wall_left {wall_left}')
    print(f'Wall_right {wall_right}')

def convert_to_V(ssR, ssL):
    # Assuming the sensor value (ssR, ssL) needs to be converted to voltage.
    ad_data_vo_ssr = (ssR * 3.3) / 1023
    ad_data_vo_ssl = (ssL * 3.3) / 1023
    return ad_data_vo_ssr, ad_data_vo_ssl

def convert_to_cm(voltage):
    if 2.2 <= voltage < 3.2:
        cm = (voltage - 4.30764) / -0.3846
    elif 1.4 <= voltage < 2.2:
        cm = (voltage - 3.2) / -0.2
    elif 0.8 <= voltage < 1.4:
        cm = (voltage - 1.87) / -0.067
    elif 0.4 <= voltage < 0.8:
        cm = (voltage - 1.344) / -0.034
    else:
        if voltage >= 3.2:
            cm = (voltage - 4.30764) / -0.3846
        elif voltage < 0.4:
            cm = (voltage - 1.344) / -0.034
    
    return cm

# ฟังก์ชันตรวจสอบเส้นทางข้างหน้า
def front_wall():
    print("front_wall called")
    if tof_distance is not None:
        return tof_distance < 250
    return False

# ฟังก์ชันตรวจสอบทางด้านซ้าย
def check_wall_left(io_data): #sensor ir
    ir_left = io_data[3]  # เซ็นเซอร์ซ้าย
    if ir_left == 0:
        print('Wall left')
        return True
    else:
        return False

# ฟังก์ชันตรวจสอบทางด้านขวา
def check_wall_right(io_data): #sensor ir
    ir_right = io_data[2]  # เซ็นเซอร์ขวา
    if ir_right == 0:
        print('Wall right')
        return True
    else:
        return False

# ฟังก์ชันการแก้ปัญหาเขาวงกตด้วย DFS
def dfs_solve():
    print("dfs_solve called")
    stack = []
    visited = set()
    start_position = (0, 0)  # Assuming starting at (0, 0)
    stack.append(start_position)
    visited.add(start_position)

    while stack:
        current_position = stack.pop()
        x, y = current_position
        print(f"Current position: {current_position}")

        # Move forward if no front wall
        if not front_wall():
            move_forward()
            new_position = (x + 1, y)
            if new_position not in visited:
                stack.append(new_position)
                visited.add(new_position)
        else:
            # Try to turn right
            if not check_wall_right():
                turn_right()
                move_forward()
                new_position = (x, y + 1)
                if new_position not in visited:
                    stack.append(new_position)
                    visited.add(new_position)
            # Try to turn left
            elif not check_wall_left():
                turn_left()
                move_forward()
                new_position = (x, y - 1)
                if new_position not in visited:
                    stack.append(new_position)
                    visited.add(new_position)
            else:
                turn_around()

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
            dfs_solve()
        except KeyboardInterrupt:
            print("Process interrupted")
        finally:
            ep_sensor.unsub_adapter()
            ep_tof.unsub_adapter()
            ep_chassis.unsub_position()
            ep_robot.close()