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

    print("Current direction: {}".format(direction))
    









# เริ่มต้นโปรแกรมหลัก
if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis


    ep_sensor = ep_robot.sensor_adaptor
    ep_sensor.sub_adapter(freq=10, callback=sub_data_handler)  # เรียกใช้ callback ทุก 5 ครั้งต่อวินาที
    ep_chassis.sub_attitude(freq=10, callback=sub_attitude_info_handler)
    time.sleep(200)  # ใช้งานเซ็นเซอร์ 200 วินาที
    ep_sensor.unsub_adapter()
    ep_chassis.unsub_attitude()
    ep_robot.close()
