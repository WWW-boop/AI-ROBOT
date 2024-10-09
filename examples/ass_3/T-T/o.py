from robomaster import robot
import time
import random

# Global variables
current_position = (3, 3)  # Starting position
direction_facing = 'N'  # Initial direction
visited_positions = []
max_exploration_time = 600  # 10 minutes
start_time = time.time()

# Sensor data
tof_distance = None
ir_left = None
ir_right = None
dis_ssL = None
dis_ssR = None

def move_forward():
    ep_robot.chassis.move(x=0.6, y=0, z=0, xy_speed=0.5).wait_for_completed()
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()
    print("------- move forward -------")

def turn_left():
    ep_robot.chassis.move(x=0, y=0, z=90, z_speed=100).wait_for_completed()
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()
    update_direction('left')
    print("------- turn left -------")

def turn_right():
    ep_robot.chassis.move(x=0, y=0, z=-91, z_speed=100).wait_for_completed()
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()
    update_direction('right')
    print("------- turn right -------")

def turn_around():
    ep_robot.chassis.move(x=0, y=0, z=180, z_speed=100).wait_for_completed()
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()
    update_direction('back')
    print("------- turn around -------")

def stop_moving():
    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    time.sleep(0.5)

def update_direction(turn):
    global direction_facing
    directions = ['N', 'E', 'S', 'W']
    current_index = directions.index(direction_facing)
    if turn == 'left':
        direction_facing = directions[(current_index - 1) % 4]
    elif turn == 'right':
        direction_facing = directions[(current_index + 1) % 4]
    elif turn == 'back':
        direction_facing = directions[(current_index + 2) % 4]

def tof_data_handler(sub_info):
    global tof_distance
    tof_distance = sub_info[0]

def sub_data_handler(sub_info):
    global ir_left, ir_right, dis_ssL, dis_ssR
    io_data, ad_data = sub_info
    ir_right = io_data[2]
    ir_left = io_data[3]
    dis_ssR = convert_to_cm(convert_to_V(ad_data[1])) / 2
    dis_ssL = convert_to_cm(convert_to_V(ad_data[0])) / 2

def sub_attitude_handler(attitude_info):
    yaw, pitch, roll = attitude_info
    global direction_facing
    if -45 < yaw <= 45:
        direction_facing = 'N'
    elif 45 < yaw < 135:
        direction_facing = 'E'
    elif 135 < yaw < 180 or -180 < yaw < -135:
        direction_facing = 'S'
    elif -135 < yaw <= -45:
        direction_facing = 'W'
    print(f"Current direction: {direction_facing}")

def convert_to_V(sensor_value):
    return (sensor_value * 3.3) / 1023

def convert_to_cm(voltage):
    if voltage >= 3.2:
        return (voltage - 4.30764) / -0.3846
    elif 2.2 <= voltage < 3.2:
        return (voltage - 4.30764) / -0.3846
    elif 1.4 <= voltage < 2.2:
        return (voltage - 3.2) / -0.2
    elif 0.8 <= voltage < 1.4:
        return (voltage - 1.87) / -0.067
    elif 0.4 <= voltage < 0.8:
        return (voltage - 1.344) / -0.034
    else:
        return (voltage - 1.344) / -0.034

def front_wall():
    return tof_distance is not None and tof_distance < 350

def check_wall_left():
    return ir_left == 0

def check_wall_right():
    return ir_right == 0

def adjust_position():
    if front_wall():
        err_dis_f = (tof_distance - 200) / 1000
        if abs(err_dis_f) >= 0.01:
            ep_robot.chassis.move(x=err_dis_f, y=0, z=0, xy_speed=0.5).wait_for_completed()
    
    if check_wall_left() and check_wall_right():
        if abs(dis_ssL - dis_ssR) >= 0.1:
            move = (dis_ssL - dis_ssR) / 2 / 100
            ep_robot.chassis.move(x=0, y=move, z=0, xy_speed=0.5).wait_for_completed()
    elif check_wall_left():
        err_dis_l = (dis_ssL - 23) / 100
        if abs(err_dis_l) >= 0.01:
            ep_robot.chassis.move(x=0, y=err_dis_l/100, z=0, xy_speed=0.5).wait_for_completed()
    elif check_wall_right():
        err_dis_r = (dis_ssR - 23) / 100
        if abs(err_dis_r) >= 0.01:
            ep_robot.chassis.move(x=0, y=-err_dis_r/100, z=0, xy_speed=0.5).wait_for_completed()

def update_position():
    global current_position
    x, y = current_position
    if direction_facing == 'N':
        current_position = (x, y + 1)
    elif direction_facing == 'E':
        current_position = (x + 1, y)
    elif direction_facing == 'S':
        current_position = (x, y - 1)
    elif direction_facing == 'W':
        current_position = (x - 1, y)
    visited_positions.append(current_position)
    print(f"Current Position: {current_position}")

def choose_direction():
    x, y = current_position
    visits = {
        'N': visited_positions.count((x, y+1)),
        'S': visited_positions.count((x, y-1)),
        'E': visited_positions.count((x+1, y)),
        'W': visited_positions.count((x-1, y))
    }
    
    # Remove directions with walls
    if front_wall():
        visits.pop(direction_facing, None)
    if check_wall_left():
        left_direction = ['W', 'N', 'E', 'S'][(['N', 'E', 'S', 'W'].index(direction_facing) - 1) % 4]
        visits.pop(left_direction, None)
    if check_wall_right():
        right_direction = ['E', 'S', 'W', 'N'][(['N', 'E', 'S', 'W'].index(direction_facing) + 1) % 4]
        visits.pop(right_direction, None)
    
    if not visits:
        return random.choice(['N', 'E', 'S', 'W'])
    return min(visits, key=visits.get)

def turn_to_direction(target_direction):
    while direction_facing != target_direction:
        turn_right()

def check_exit_condition():
    return time.time() - start_time > max_exploration_time

def maze_explored():
    while not check_exit_condition():
        print('maze_explored')
        print(f"Visited positions: {visited_positions}")
        
        next_direction = choose_direction()
        turn_to_direction(next_direction)
        
        if not front_wall():
            adjust_position()
            move_forward()
            update_position()
        else:
            continue
        
        if check_exit_condition():
            print("Exploration time limit reached.")
            break

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis
    ep_sensor = ep_robot.sensor_adaptor
    ep_tof = ep_robot.sensor
    ep_gimbal = ep_robot.gimbal

    ep_chassis.sub_attitude(freq=10, callback=sub_attitude_handler)
    ep_sensor.sub_adapter(freq=10, callback=sub_data_handler)
    ep_tof.sub_distance(freq=10, callback=tof_data_handler)

    ep_gimbal.recenter().wait_for_completed()
    time.sleep(3)

    try:
        maze_explored()
    except KeyboardInterrupt:
        print("Process interrupted by user")
    finally:
        ep_tof.unsub_distance()
        ep_sensor.unsub_adapter()
        ep_chassis.unsub_attitude()
        ep_robot.close()

    print("Maze exploration completed.")
    print(f"Final visited positions: {visited_positions}")