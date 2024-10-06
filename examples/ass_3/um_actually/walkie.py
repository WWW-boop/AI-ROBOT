import robomaster 
from robomaster import robot
import time
import matplotlib.pyplot as plt
import csv
from datetime import datetime
from collections import deque

def sub_attitude_info_handler(attitude_info):
    yaw, pitch, roll = attitude_info
# เริ่มต้นหุ่นยนต์ RoboMaster
ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")  # เชื่อมต่อหุ่นยนต์ด้วยโหมด ap (access point)
ep_chassis = ep_robot.chassis
ep_chassis.sub_attitude(freq=10, callback=sub_attitude_info_handler)

# กำหนดขนาดของตาราง (6x6 grid)
GRID_SIZE = 6  # ขนาดของกริดคือ 20x20
grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]  # กำหนดค่า 0 สำหรับช่องว่าง, 1 สำหรับกำแพง, 3 สำหรับเส้นทางที่ผ่าน
robot_position = (5, 5)  # ตำแหน่งเริ่มต้นของหุ่นยนต์ที่พิกัด (0, 0)
DIRECTIONS = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # ทิศทางในการเดิน: ขวา, ลง, ซ้าย, ขึ้น

# ตัวแปรข้อมูลเซ็นเซอร์ระดับโลก
tof_distance = None  # ข้อมูลเซ็นเซอร์ ToF (Time of Flight) ด้านหน้า
dis_ssL = None  # ระยะทางจากเซ็นเซอร์ซ้าย
dis_ssR = None  # ระยะทางจากเซ็นเซอร์ขวา
wall_left = None  # ตรวจสอบว่ามีกำแพงทางซ้ายหรือไม่
wall_right = None  # ตรวจสอบว่ามีกำแพงทางขวาหรือไม่

# ฟังก์ชันแสดงผลตาราง
def draw_grid():
    plt.imshow(grid, cmap='Blues', vmin=0, vmax=3)  # แสดงผลกริดในแผนที่สี "Blues"
    plt.grid(True)  # แสดงเส้นตาราง
    plt.xticks(range(GRID_SIZE))  # กำหนดขนาดแกน X
    plt.yticks(range(GRID_SIZE))  # กำหนดขนาดแกน Y
    plt.pause(0.5)  # พักเพื่อจำลองการอัปเดตแบบเรียลไทม์

# การสำรวจแบบ DFS (Depth-First Search) ด้วยการวาดเส้นทาง
def dfs_explore(position):
    print(f"Exploring from {position}")  # พิมพ์ว่ากำลังสำรวจตำแหน่งใด
    stack = [position]  # ใช้ stack สำหรับ DFS โดยเริ่มจากตำแหน่งปัจจุบัน
    grid[position[0]][position[1]] = 2  # กำหนดตำแหน่งเริ่มต้นของหุ่นยนต์เป็นค่า 2
    draw_grid()  # วาดกริดใหม่

    while stack:
        current = stack.pop()  # นำตำแหน่งปัจจุบันออกจาก stack
        grid[current[0]][current[1]] = 3  # กำหนดให้ตำแหน่งที่ผ่านแล้วเป็นค่า 3
        draw_grid()  # วาดกริดใหม่

        # อัปเดตกริดตามข้อมูลเซ็นเซอร์ (ตรวจจับกำแพงหรือสิ่งกีดขวาง)
        update_grid_with_sensors()

        # สำรวจทิศทางต่างๆ (ขวา, ลง, ซ้าย, ขึ้น)
        for direction in DIRECTIONS:
            neighbor = (current[0] + direction[0], current[1] + direction[1])  # คำนวณตำแหน่งถัดไปในทิศทางที่กำหนด

            # ตรวจสอบว่าตำแหน่งถัดไปอยู่ในกรอบและยังไม่เคยเยือน
            if 0 <= neighbor[0] < GRID_SIZE and 0 <= neighbor[1] < GRID_SIZE:
                if grid[neighbor[0]][neighbor[1]] == 0:  # หากตำแหน่งนั้นเป็นทางที่ยังไม่เยือนและเป็นทางว่าง
                    # เคลื่อนหุ่นยนต์ไปในทิศทางที่กำหนด
                    if direction == (0, 1):  # เคลื่อนไปทางขวา
                        if not wall_right:
                            turn_right()  # ตรวจสอบว่ามีกำแพงทางขวาหรือไม่
                            move_forward()
                            
                    elif direction == (1, 0):  # เคลื่อนไปทางล่าง
                        if not front_wall():  # ตรวจสอบว่ามีกำแพงด้านหน้า
                            move_forward()
                        turn_left()  # หมุนกลับเพื่อรีเซ็ตทิศทาง
                    elif direction == (0, -1):  # เคลื่อนไปทางซ้าย
                        if not wall_left:  # ตรวจสอบว่ามีกำแพงทางซ้ายหรือไม่
                            turn_left()  # หมุนซ้าย
                            move_forward()  # เคลื่อนที่ไปข้างหน้า
                              # หมุนขวาเพื่อรีเซ็ตทิศทาง
                    elif direction == (-1, 0):  # เคลื่อนไปทางขึ้น
                        turn_around()  # หมุนหุ่นยนต์ 180 องศา
                        if not front_wall():  # ตรวจสอบว่ามีกำแพงด้านหน้าหรือไม่
                            move_forward()  # เคลื่อนที่ไปข้างหน้า
                        turn_around()  # หมุนกลับเพื่อรีเซ็ตทิศทาง
                    
                    # ทำเครื่องหมายตำแหน่งใหม่เป็นตำแหน่งปัจจุบันของหุ่นยนต์
                    grid[neighbor[0]][neighbor[1]] = 2
                    stack.append(neighbor)  # เพิ่มตำแหน่งใหม่ลงใน stack
                    draw_grid()  # วาดกริดใหม่

# อัปเดตกริดตามข้อมูลเซ็นเซอร์
def update_grid_with_sensors():
    print("update_grid_with_sensors called")  # พิมพ์ว่าได้เรียกใช้ฟังก์ชันอัปเดตข้อมูลเซ็นเซอร์
    global robot_position  # ใช้ตัวแปรตำแหน่งของหุ่นยนต์ที่เป็น global
    global grid  # ใช้ตัวแปรกริดที่เป็น global

    # สมมติว่าหุ่นยนต์สามารถตรวจจับสิ่งกีดขวางได้เพียง 1 ช่องรอบๆ
    if front_wall():  # ถ้าด้านหน้ามีกำแพง
        front_pos = (robot_position[0], robot_position[1] + 1)  # คำนวณตำแหน่งด้านหน้า
        if front_pos[1] < GRID_SIZE:  # ตรวจสอบว่าตำแหน่งอยู่ในขอบเขตของกริด
            grid[front_pos[0]][front_pos[1]] = 1  # ทำเครื่องหมายตำแหน่งเป็นกำแพง
    
    if wall_left:  # ถ้ามีกำแพงทางซ้าย
        left_pos = (robot_position[0], robot_position[1] - 1)  # คำนวณตำแหน่งทางซ้าย
        if left_pos[1] >= 0:  # ตรวจสอบว่าตำแหน่งอยู่ในขอบเขตของกริด
            grid[left_pos[0]][left_pos[1]] = 1  # ทำเครื่องหมายตำแหน่งเป็นกำแพง
    
    if wall_right:  # ถ้ามีกำแพงทางขวา
        right_pos = (robot_position[0], robot_position[1] + 1)  # คำนวณตำแหน่งทางขวา
        if right_pos[1] < GRID_SIZE:  # ตรวจสอบว่าตำแหน่งอยู่ในขอบเขตของกริด
            grid[right_pos[0]][right_pos[1]] = 1  # ทำเครื่องหมายตำแหน่งเป็นกำแพง

# ฟังก์ชันเคลื่อนที่ไปข้างหน้า
def move_forward():
    for i in range(2):  # เคลื่อนที่ไปข้างหน้า 2 ครั้ง
        print("move_forward called")  # พิมพ์ว่ากำลังเรียก move_forward
        ep_robot.chassis.move(x=0.3, y=0, z=0, xy_speed=10).wait_for_completed()  # เคลื่อนที่ไปข้างหน้าด้วยความเร็วที่กำหนด
        ep_robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        ep_gimbal.recenter().wait_for_completed()  # หยุดการเคลื่อนที่ของล้อ

# ฟังก์ชันหมุนซ้าย
def turn_left():
    print("turn_left called")  # พิมพ์ว่ากำลังเรียก turn_left
    ep_robot.chassis.move(x=0, y=0, z=90, z_speed=100).wait_for_completed()  # หมุนซ้าย 90 องศา
    ep_robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    ep_gimbal.recenter().wait_for_completed()  # หยุดการหมุนของล้อ

# ฟังก์ชันหมุนขวา
def turn_right():
    print("turn_right called")  # พิมพ์ว่ากำลังเรียก turn_right
    ep_robot.chassis.move(x=0, y=0, z=-91, z_speed=100).wait_for_completed()  # หมุนขวา 91 องศา
    ep_robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    ep_gimbal.recenter().wait_for_completed()  # หยุดการหมุนของล้อ
    
def turn_around():
    print("turn_around called")
    ep_robot.chassis.move(x=0, y=0, z=180, z_speed=100).wait_for_completed()
    ep_robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    ep_gimbal.recenter().wait_for_completed()

# ToF sensor handler (front)
def tof_data_handler(sub_info):
    print("tof_data_handler called")
    distance = sub_info
    global tof_distance
    tof_distance = distance[0]
    print(f"ToF: {distance[0]}")

# Sensor data handler (left and right)
def sub_data_handler(sub_info): 
    io_data, ad_data = sub_info
    global dis_ssL, dis_ssR, wall_right, wall_left

    # กรองค่า ADC ก่อน (Filter ADC values)
    smoothed_values = filter_ad_data(ad_data)
    
    # แปลงค่า ADC เป็นแรงดันไฟฟ้า (Convert ADC to voltage)
    ssR = smoothed_values[1] 
    ssL = smoothed_values[0] 
    vaR, vaL = convert_to_V(ssR, ssL)

    # แปลงค่าแรงดันไฟฟ้าเป็นระยะทาง (Convert voltage to distance)
    dis_ssR = convert_to_cm(vaR) / 2
    dis_ssL = convert_to_cm(vaL) / 2 

    print(f"Distance ssR: {dis_ssR} cm")
    print(f"Distance ssL: {dis_ssL} cm")
    
    # ตรวจสอบกำแพง (Check walls)
    wall_left = check_wall_left(io_data)
    wall_right = check_wall_right(io_data)

    print(f'Wall_left {wall_left}')
    print(f'Wall_right {wall_right}')

def filter_ad_data(ad_data):
    filtered_data = []
    smoothing_factor = 0.1  # Smoothing factor
    previous_value = 0

    for reading in ad_data:
        current_value = smoothing_factor * previous_value + (1 - smoothing_factor) * reading
        filtered_data.append(current_value)
        previous_value = current_value

    return filtered_data

def convert_to_V(ssR, ssL):
    ad_data_vo_ssr = (ssR * 3.3) / 1023
    ad_data_vo_ssl = (ssL * 3.3) / 1023
    return ad_data_vo_ssr, ad_data_vo_ssl

def convert_to_cm(voltage):
    # Conversion formula from voltage to distance
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
def front_wall():
    if tof_distance is not None:
        return tof_distance < 250  # Adjust distance threshold based on testing
    return False

def check_wall_left(io_data):  # IR sensor check for left wall
    ir_left = io_data[3]  # เซ็นเซอร์ซ้าย
    if ir_left == 0:
        print('Wall left')
        return True
    return False

def check_wall_right(io_data):  # IR sensor check for right wall
    ir_right = io_data[2]  # เซ็นเซอร์ขวา
    if ir_right == 0:
        print('Wall right')
        return True
    return False

def sub_position_handler(position_info):
    x, y, z = position_info
    elapsed_time = time.time() - start_time
    print(f"chassis position: x:{x}, y:{y}, time:{elapsed_time}")
    writer.writerow([x, y, elapsed_time])
    file.flush()  # Ensure data is written to the file immediately
    x_data.append(x)
    y_data.append(y)

# Main function to explore every path
if __name__ == '__main__':
    x_data = []
    y_data = []
    start_time = time.time()

    plt.ion()  # Enable interactive mode for live updating plot
    fig, ax = plt.subplots()

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
            dfs_explore(robot_position)  # Start exploring from initial position
        except KeyboardInterrupt:
            print("Process interrupted")
        finally:
            ep_sensor.unsub_adapter()
            ep_tof.unsub_adapter()
            ep_chassis.unsub_position()
            ep_robot.close()
            plt.ioff()  # Disable interactive mode
            plt.show()  # Show final grid