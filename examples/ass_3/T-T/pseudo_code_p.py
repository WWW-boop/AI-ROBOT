import robomaster
from robomaster import robot
import time

# เริ่มต้นหุ่นยนต์ RoboMaster
ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")

def filter_ad_data(ad_data):
    filtered_data = []
    smoothing_factor = 0.1  
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

# 
# เดินไปข้างหน้า
#     เคลื่อนที่ไป 1 กระเบื้อง
def move_forward():
    for i in range(2):
        ep_robot.chassis.move(x=0.3, y=0, z=0, xy_speed=10).wait_for_completed()
        ep_robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

# ----------------------
# หันซ้าย
#     หันไปท้างซ้าย
def turn_left():
    ep_robot.chassis.move(x=0, y=0, z=90, z_speed=80).wait_for_completed()
    ep_robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)


# --------------------
# หันขวา
#     หันไปทางขวา
def turn_right():
    ep_robot.chassis.move(x=0, y=0, z=-91, z_speed=80).wait_for_completed()
    ep_robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

# --------------------
# กลับหลังหัน
#     หันกลับไปทางข้างหลัง
def turn_around():
    ep_robot.chassis.move(x=0, y=0, z=180, z_speed=80).wait_for_completed()
    ep_robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

# ----------------------
# ตรวจกำแพงซ้าย
#     ถ้าเจอกำแพงซ้าย
#         จริง
#     ถ้าไม่
#         ไม่จริง
def check_wall_left(io_data): #sensor ir
    ir_left = io_data[3]  # เซ็นเซอร์ซ้าย
    if ir_left == 0:
        print('Wall left')
        return True
    else:
        return False

# ---------------
# ตรวจกำแพงขวา
#     ถ้าเจอกำแพงขวา
#         จริง
#     ถ้าไม่
#         ไม่จริง
def check_wall_right(io_data): #sensor ir
    ir_right = io_data[2]  # เซ็นเซอร์ขวา
    if ir_right == 0:
        print('Wall right')
        return True
    else:
        return False

# ---------------
# ตรวจกำแพงหน้า
#     ถ้าเจอกำแพงหน้า
#         จริง
#     ถ้าไม่
#         ไม่จริง
def front_wall_tof(sub_info): #เอาไว้เช็คข้างหน้าว่ามีกำแพงไหม
    distance = sub_info
    if distance < 350:
        return True
    return False


# ----------------
# กำหนดทิศ
#     ทิศที่กำลังมอง = 0
#     องศา == attitude.yaw
#     ถ้า  -45 < องศา < 45 
#         ทิศที่กำลังมอง = 'N'
#     ถ้า 45 < องศา < 135
#         ทิศที่กำลังมอง = 'E'
#     ถ้า 135 < องศา < 180 or -180 < องศา < -135
#         ทิศที่กำลังมอง = 'S'
#     ถ้า -135 < องศา < -45
#         ทิศที่กำลังมอง = 'W'
def sub_attitude_info_handler(attitude_info):
    yaw = attitude_info  # รับค่าจาก attitude_info
    

    # กำหนดทิศตามค่า yaw
    if -45 < yaw < 45:
        direction = 'N'  # ทิศเหนือ
    elif 45 <= yaw < 135:
        direction = 'E'  # ทิศตะวันออก
    elif 135 <= yaw < 180 or -180 < yaw <= -135:
        direction = 'S'  # ทิศใต้
    elif -135 < yaw < -45:
        direction = 'W'  # ทิศตะวันตก
    else:
        direction = 'Unknown'  # กรณีที่ไม่ตรงกับเงื่อนไขใดๆ

    return direction
    
# เริ่มการสำรวจ maze
visited_positions = set()
intersections = []

x, y = 0, 0  # จุดเริ่มต้น
direction = 'N'  # เริ่มจากมองไปทิศเหนือ

# ฟังก์ชันตรวจสอบตำแหน่งที่เดินด้วย DFS
def dfs_walk(x, y, direction, visited_positions, intersections):
    if direction == 'N':
        if not front_wall_tof():  # ตรวจสอบว่าด้านหน้าว่าง
            move_forward()
            y += 1  # เดินไปทางทิศเหนือ
            visited_positions.add((x, y))
        elif not check_wall_left():  # ถ้าไม่มีผนังด้านซ้าย
            turn_left()
            move_forward()
            x -= 1  # ไปทางซ้าย (ทิศตะวันตก)
            visited_positions.add((x, y))
            direction = 'W'
        elif not check_wall_right():  # ถ้าไม่มีผนังด้านขวา
            turn_right()
            move_forward()
            x += 1  # ไปทางขวา (ทิศตะวันออก)
            visited_positions.add((x, y))
            direction = 'E'
        else:
            turn_around()  # ถ้าเจอผนังทั้งสามด้าน
            backtrack(intersections)  # เดินถอยหลังไปจุดแยกก่อนหน้า

    elif direction == 'E':
        if not front_wall_tof():
            move_forward()
            x += 1  # เดินไปทางทิศตะวันออก
            visited_positions.add((x, y))
        elif not check_wall_left():
            turn_left()
            move_forward()
            y += 1  # ไปทางซ้าย (ทิศเหนือ)
            visited_positions.add((x, y))
            direction = 'N'
        elif not check_wall_right():
            turn_right()
            move_forward()
            y -= 1  # ไปทางขวา (ทิศใต้)
            visited_positions.add((x, y))
            direction = 'S'
        else:
            turn_around()
            backtrack(intersections)

    elif direction == 'S':
        if not front_wall_tof():
            move_forward()
            y -= 1  # เดินไปทางทิศใต้
            visited_positions.add((x, y))
        elif not check_wall_left():
            turn_left()
            move_forward()
            x += 1  # ไปทางซ้าย (ทิศตะวันออก)
            visited_positions.add((x, y))
            direction = 'E'
        elif not check_wall_right():
            turn_right()
            move_forward()
            x -= 1  # ไปทางขวา (ทิศตะวันตก)
            visited_positions.add((x, y))
            direction = 'W'
        else:
            turn_around()
            backtrack(intersections)

    elif direction == 'W':
        if not front_wall_tof():
            move_forward()
            x -= 1  # เดินไปทางทิศตะวันตก
            visited_positions.add((x, y))
        elif not check_wall_left():
            turn_left()
            move_forward()
            y -= 1  # ไปทางซ้าย (ทิศใต้)
            visited_positions.add((x, y))
            direction = 'S'
        elif not check_wall_right():
            turn_right()
            move_forward()
            y += 1  # ไปทางขวา (ทิศเหนือ)
            visited_positions.add((x, y))
            direction = 'N'
        else:
            turn_around()
            backtrack(intersections)

# ฟังก์ชัน backtrack เมื่อไม่มีทางไป
def backtrack(intersections):
    # กลับไปจุดแยกล่าสุด
    if intersections:
        intersection = intersections.pop()
        x, y = intersection
        # เดินกลับไปยังตำแหน่งนั้น
        move_to(x, y)

def move_to(x, y, current_x, current_y, visited_positions, direction_stack):
    while current_x != x or current_y != y:
        # ใช้ข้อมูลจาก stack เพื่อติดตามทิศทางที่เคยเดิน
        last_direction = direction_stack.pop()  # ดึงทิศทางล่าสุดจาก stack

        if last_direction == 'N':
            turn_around()  # ถ้าเคยเดินขึ้นเหนือ, ต้องหมุนกลับไปทิศใต้
            move_forward()  # เดินกลับหนึ่งก้าว
            current_y -= 1  # ปรับตำแหน่ง y เมื่อถอยกลับ
        elif last_direction == 'E':
            turn_around()  # ถ้าเคยเดินไปตะวันออก, ต้องหมุนกลับทิศตะวันตก
            move_forward()
            current_x -= 1
        elif last_direction == 'S':
            turn_around()  # ถ้าเคยเดินลงใต้, ต้องหมุนกลับทิศเหนือ
            move_forward()
            current_y += 1
        elif last_direction == 'W':
            turn_around()  # ถ้าเคยเดินไปตะวันตก, ต้องหมุนกลับทิศตะวันออก
            move_forward()
            current_x += 1
        
        # เช็คว่ากลับมาถึงจุดที่ต้องการหรือยัง
        visited_positions.add((current_x, current_y))




# เริ่มต้นโปรแกรมหลัก
if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis

    dfs_walk(x, y, direction, visited_positions, intersections)
    ep_sensor = ep_robot.sensor_adaptor
    ep_sensor.sub_adapter(freq=10, callback=sub_data_handler)  # เรียกใช้ callback ทุก 5 ครั้งต่อวินาที
    ep_chassis.sub_attitude(freq=10, callback=sub_attitude_info_handler)
    ep_tof = ep_robot.sensor
    ep_tof.sub_distance(freq=10, callback=front_wall_tof)

    
    ep_sensor.unsub_adapter()
    ep_chassis.unsub_attitude()
    ep_robot.close()