import robomaster
from robomaster import robot
import time

# เริ่มต้นหุ่นยนต์ RoboMaster
ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")

# ฟังก์ชันการเคลื่อนไหว
def move_forward():
    ep_robot.chassis.move(x=0.1, y=0, z=0, xy_speed=0.5).wait_for_completed()
    ep_robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

def turn_left():
    ep_robot.chassis.move(x=0, y=0, z=90, z_speed=30).wait_for_completed()
    ep_robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

def turn_right():
    ep_robot.chassis.move(x=0, y=0, z=-96, z_speed=30).wait_for_completed()
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
    global ad_data_value
    global dis_ssL
    global dis_ssR
    ad_data_value = ad_data
    ssR = ad_data_value[2]
    ssL = ad_data_value[3]

    vaR, vaL = convert_to_V(ssR, ssL)
    dis_ssR = convert_to_cm(vaR)
    dis_ssL = convert_to_cm(vaL)
    print(f"Distance ssR: {dis_ssR}  cm")
    print(f"Distance ssL: {dis_ssL}  cm")

def convert_to_V(ssR,ssL):
    # Assuming the sensor value (ssR) needs to be converted to centimeters.
    ad_data_vo_ssr = (ssR * 3) / 1023
    ad_data_vo_ssl = (ssL * 3) / 1023
    return ad_data_vo_ssr , ad_data_vo_ssl

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
    if tof_distance is not None:
        return tof_distance < 230  # ถ้าต่ำกว่า 200 คือเจอกำแพง
    return False

# ฟังก์ชันตรวจสอบทางด้านซ้าย
def left_wall():
    if dis_ssL < 25 :
        return True 
    return False

# ฟังก์ชันตรวจสอบทางด้านขวา
def right_wall():
    if dis_ssR < 35 :
        return True 
    return False

def stick_right_wall():
    global err_dis_r
    err_dis_r = (dis_ssR-18)/100
    if abs(err_dis_r) >= 0.1:
        return True
    return False
        

# ฟังก์ชันการแก้ปัญหาเขาวงกต
def wall_following_solve():
    time.sleep(1)
    while True:
        time.sleep(0.5)
        if right_wall(): #ถ้าเจอกำแพงด้านขวา
            if stick_right_wall() == True:
                ep_robot.chassis.move(x=0, y=err_dis_r, z=0, xy_speed=1).wait_for_completed()
                ep_robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
            if  front_wall() == False: #ถ้าไม่พบกำแพงข้างหน้า
                move_forward() # เคลื่อนที่ไปข้างหน้า 1 กระเบื้อง
            elif front_wall(): # ถ้าพบกำแพงทั้งข้างหน้าและขวา
                turn_left() #ให้หันซ้าย
        elif right_wall() == False: #ถ้าไม่พบกำแพงขวา
            for i in range(3):
                move_forward()
            turn_right() # ให้หันขวาและตรงไป 
            ep_gimbal.recenter().wait_for_completed()
            for i in range(7):
                move_forward()
            if right_wall() == False:
                turn_right()
                ep_gimbal.recenter().wait_for_completed()
                for i in range(7):
                    move_forward()
        ep_gimbal.recenter().wait_for_completed() #ขยับgimbal มาตรงกลางทุกครั้งที่ทำตามคำสั่งข้างต้น
        #จากเงื่อนไขและคำสั่งข้างต้นจะทำให้หุ่นจะเดินตามกำแพงขวาไปเรื่อยๆ
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
