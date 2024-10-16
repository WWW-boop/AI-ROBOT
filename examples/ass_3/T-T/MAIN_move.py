from robomaster import robot
#from robot_detector import RobotDetector
import time
import matplotlib.pyplot as plt
import numpy as np

# Constants for the map
MAP_SIZE = 6
BLOCK_SIZE = 60  # Assuming each block is 60cm x 60cm (adjust as needed)

# Initialize the map
grid_map = [[0 for _ in range(MAP_SIZE)] for _ in range(MAP_SIZE)]

# Current position and direction of the robot
current_position = [3,1]
current_direction = 'N'  # 'N', 'E', 'S', 'W'++++

def initialize_map():
    # Mark the edges as walls (1 represents a wall)
    for i in range(MAP_SIZE):
        grid_map[0][i] = 1
        grid_map[MAP_SIZE-1][i] = 1
        grid_map[i][0] = 1
        grid_map[i][MAP_SIZE-1] = 1

def is_valid_move(x, y):
    return 0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE and grid_map[y][x] == 0

def update_position(direction):
    global current_position, current_direction
    x, y = current_position
    
    if direction == 'forward':
        if current_direction == 'N' and is_valid_move(x, y-1):
            current_position = [x, y-1]
        elif current_direction == 'E' and is_valid_move(x+1, y):
            current_position = [x+1, y]
        elif current_direction == 'S' and is_valid_move(x, y+1):
            current_position = [x, y+1]
        elif current_direction == 'W' and is_valid_move(x-1, y):
            current_position = [x-1, y]
    
    elif direction == 'left':
        current_direction = {'N': 'W', 'W': 'S', 'S': 'E', 'E': 'N'}[current_direction]
    
    elif direction == 'right':
        current_direction = {'N': 'E', 'E': 'S', 'S': 'W', 'W': 'N'}[current_direction]
    
    elif direction == 'around':
        current_direction = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}[current_direction]


def update_map():
    x, y = current_position
    
    # Update the current position as visited
    grid_map[y][x] = 2  # 2 represents visited cells
    
    # Check and update surrounding cells based on current direction
    if current_direction == 'N':
        if front_wall():
            grid_map[y-1][x] = 1 if y-1 >= 0 else grid_map[y-1][x]
        if check_wall_left():
            grid_map[y][x-1] = 1 if x-1 >= 0 else grid_map[y][x-1]
        if check_wall_right():
            grid_map[y][x+1] = 1 if x+1 < MAP_SIZE else grid_map[y][x+1]
    elif current_direction == 'E':
        if front_wall():
            grid_map[y][x+1] = 1 if x+1 < MAP_SIZE else grid_map[y][x+1]
        if check_wall_left():
            grid_map[y-1][x] = 1 if y-1 >= 0 else grid_map[y-1][x]
        if check_wall_right():
            grid_map[y+1][x] = 1 if y+1 < MAP_SIZE else grid_map[y+1][x]
    elif current_direction == 'S':
        if front_wall():
            grid_map[y+1][x] = 1 if y+1 < MAP_SIZE else grid_map[y+1][x]
        if check_wall_left():
            grid_map[y][x+1] = 1 if x+1 < MAP_SIZE else grid_map[y][x+1]
        if check_wall_right():
            grid_map[y][x-1] = 1 if x-1 >= 0 else grid_map[y][x-1]
    elif current_direction == 'W':
        if front_wall():
            grid_map[y][x-1] = 1 if x-1 >= 0 else grid_map[y][x-1]
        if check_wall_left():
            grid_map[y+1][x] = 1 if y+1 < MAP_SIZE else grid_map[y+1][x]
        if check_wall_right():
            grid_map[y-1][x] = 1 if y-1 >= 0 else grid_map[y-1][x]

def print_map():
    for row in grid_map:
        print(' '.join(str(cell) for cell in row))
    print(f"Current position: {current_position}, Direction: {current_direction}")

# ------------------- action -----------------------
# --------------------------------------------------

def move_forward():
    
    ep_robot.chassis.move(x=0.6, y=0, z=0, xy_speed=0.7).wait_for_completed()
    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()
    print("------- move forward -------")

# --------------------------------------------------

def turn_left():
    
    ep_robot.chassis.move(x=0, y=0, z=90, z_speed=100).wait_for_completed()
    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()
    print("------- turn left -------")

# --------------------------------------------------

def turn_right():
    
    ep_robot.chassis.move(x=0, y=0, z=-91, z_speed=100).wait_for_completed()
    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()
    # update_direction('right')
    print("------- turn right -------")

# --------------------------------------------------
    
def turn_around():

    ep_robot.chassis.move(x=0, y=0, z=180, z_speed=100).wait_for_completed()
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()
    # update_direction('back')
    print("------- turn around -------")

# --------------------------------------------------

def stop_moving():
    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    time.sleep(0.5)

# --------------------------------------------------

# ------------------- data -------------------------

def tof_data_handler(sub_info):
    global tof_distance
    distance = sub_info
    tof_distance = distance[0]
# --------------------------------------------------

def filter_ad_data(ad_data):
    filtered_data = []
    smoothing_factor = 0.1  # ค่าอัลฟาสำหรับการกรอง
    previous_value = 0  # กำหนดค่าเริ่มต้นของ previous_value

    for reading in ad_data:
        current_value = smoothing_factor * previous_value + (1 - smoothing_factor) * reading
        filtered_data.append(current_value)
        previous_value = current_value

    return filtered_data

# --------------------------------------------------

def sub_data_handler(sub_info):  # sensor sharp เอาไว้เช็คกำแพงว่าห่างจาก กำแพงทำไร แล้วปรับค่าให้อยู่ตรงกลางตลอดของกำแพง
    global io_data, dis_ssL, dis_ssR, ir_left, ir_right

    io_data, ad_data = sub_info
    smoothed_values = filter_ad_data(ad_data)

    ir_right = io_data[2]
    ir_left = io_data[3]

    ssR = smoothed_values[1] 
    ssL = smoothed_values[0] 
    vaR, vaL = convert_to_V(ssR, ssL)

    dis_ssR = convert_to_cm(vaR) / 2
    dis_ssL = convert_to_cm(vaL) / 2 

    # print(f"Distance ssR : {dis_ssR} cm")
    # print(f"Distance ssL : {dis_ssL} cm")
    print(io_data)
    

# --------------------------------------------------

def sub_attitude_handler(attitude_info):
    global yaw
    yaw, pitch, roll = attitude_info
    
def facing():
    global direction_facing
    current_yaw = yaw

    if -45 < current_yaw <= 45:
        direction_facing = 'N'

    if 45 < current_yaw < 135:
        direction_facing = 'E'

    if 135 < current_yaw < 180 or -180 < yaw < -135:
        direction_facing = 'S'

    if -135 < current_yaw <= -45:
        direction_facing = 'W'
    
    print(direction_facing)

# ---------------------------------------------------  

def convert_to_V(ssR, ssL):
    # Assuming the sensor value (ssR, ssL) needs to be converted to voltage.
    ad_data_vo_ssr = (ssR * 3.3) / 1023
    ad_data_vo_ssl = (ssL * 3.3) / 1023
    return ad_data_vo_ssr, ad_data_vo_ssl

# --------------------------------------------------

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

# --------------------------------------------------

def front_wall():
    global tof_distance
    if tof_distance is None:
        print("TOF distance not available yet. Assuming no wall.")
        return False  # If no data is available, assume no wall
    if tof_distance < 350:
        print(f"Wall detected at distance: {tof_distance}")
        return True  # Wall detected
    return False  # No wall detected


# --------------------------------------------------

def check_wall_left(): #sensor ir
    if ir_left == 0:
        print('Wall left')
        return True
    return False

# --------------------------------------------------

def check_wall_right(): #sensor ir  # เซ็นเซอร์ขวา
    if ir_right == 0:
        print('Wall right')
        return True
    return False


def adjust_left(): #sensor ir
    global err_dis_l
    err_dis_l = (dis_ssL-10)/100
    if abs(err_dis_l) >= 0.01:
        ep_chassis.move(x=0, y=-err_dis_l, z=0, xy_speed=3).wait_for_completed()

# --------------------------------------------------

def adjust_right(): #sensor ir
    global err_dis_r
    err_dis_r = (dis_ssR-10)/100
    if abs(err_dis_r) >= 0.01:
        ep_chassis.move(x=0, y=err_dis_r, z=0, xy_speed=3).wait_for_completed()
# --------------------------------------------------
def adjust_position_LR():
    if abs(dis_ssL - dis_ssR) >= 10:
        if dis_ssL > dis_ssR:
            move = ((dis_ssL - dis_ssR) /2)/100
            ep_robot.chassis.move(x=0, y=-move, z=0, xy_speed=3).wait_for_completed()
        else:
            move = ((dis_ssR - dis_ssL) /2)/100
            ep_robot.chassis.move(x=0, y=move, z=0, xy_speed=3).wait_for_completed()
        


#---------------------------------------------------

def adjust_front(): #sensor ir
    global err_dis_f
    err_dis_f = (tof_distance-200)/1000
    if abs(err_dis_f) >= 0.01:
        move = err_dis_f
        ep_robot.chassis.move(x=move, y=0, z=0, xy_speed=5).wait_for_completed()

# --------------------------------------------------

def adjust_pos():
    if front_wall():
        adjust_front()
    elif check_wall_right() and not check_wall_left():    
        adjust_right()
    elif check_wall_left() and not check_wall_right():    
        adjust_left()
    elif check_wall_left() and check_wall_right():
        adjust_position_LR()

def state():
    global action_state
    initialize_map()
    time.sleep(1)
    while True:
        time.sleep(0.5)
        update_map()
        print_map()  # แสดงแผนที่หลังจากอัปเดต
        
        if front_wall() == True and check_wall_right() == True and check_wall_left() == True:
            
            adjust_pos()
            turn_around()
            update_position('around')
            move_forward()
            update_position('forward')
            action_state = 'around'
        elif front_wall() == False and check_wall_right() == True:
          
            adjust_pos()
            move_forward()
            update_position('forward')
            action_state = 'forward'
        elif check_wall_right() == False:            
            
            adjust_pos()
            turn_right()
            adjust_pos()
            update_position('right')
            move_forward()
            update_position('forward')
            action_state = 'right'
        elif front_wall() == True and check_wall_right() == True:
          
            adjust_pos()
            turn_left()
            adjust_pos()
            update_position('left')
            move_forward()
            update_position('forward')
            action_state = 'turn left'
        
        # ตรวจสอบว่าอยู่ในพื้นที่ที่กำหนดหรือไม่
        # if not is_valid_move(current_position[0], current_position[1]):
        #     print("Warning: Robot is at the edge of the map!")
        #     stop_moving()
        #     # ทำการแก้ไข เช่น ถอยกลับหรือหมุนตัว
        #     turn_around()
        #     update_position('around')
        #     move_forward()
        #     update_position('forward')


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    #detector = RobotDetector()

    ep_chassis = ep_robot.chassis
    ep_chassis.sub_attitude(freq=20, callback=sub_attitude_handler)

    ep_sensor = ep_robot.sensor_adaptor
    ep_sensor.sub_adapter(freq=20, callback=sub_data_handler)

    ep_tof = ep_robot.sensor
    ep_tof.sub_distance(freq=20, callback=tof_data_handler)

    ep_gimbal = ep_robot.gimbal
    ep_gimbal.recenter().wait_for_completed()
    time.sleep(1)

    try:
        state()
        #detector.run()
    except KeyboardInterrupt:
        print("Process interrupted by user")
    finally:
        try:
            if ep_tof:
                ep_tof.unsub_distance()
            if ep_sensor:
                ep_sensor.unsub_adapter()
            ep_chassis.unsub_attitude()
        except AttributeError as e:
            print(f"Error during unsubscribing: {e}")
        ep_robot.close()