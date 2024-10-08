from robomaster import robot
import time
import matplotlib.pyplot as plt
import numpy as np


current_position = (3, 3)  # Starting at position (3, 3)
visited_positions = []
branches = []
criminals = []

# --------------------------------------------------

def move_forward():
    
    ep_robot.chassis.move(x=0.6, y=0, z=0, xy_speed=1).wait_for_completed()
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()
    # update_direction()
    print("------- move forward -------")

# --------------------------------------------------

def turn_left():
    
    ep_robot.chassis.move(x=0, y=0, z=90, z_speed=100).wait_for_completed()
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()
    # update_direction('left')
    print("------- turn left -------")

# --------------------------------------------------

def turn_right():
    
    ep_robot.chassis.move(x=0, y=0, z=-91, z_speed=100).wait_for_completed()
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

 # Initialize globally

def tof_data_handler(sub_info):
    global tof_distance
    distance = sub_info
    tof_distance = distance[0]
     # Assuming TOF sensor data is a list, and [0] is the distance
    print(f"TOF distance: {tof_distance}")
      # Default to no wall if data is invalid
# Default to a large value indicating no wall is detected


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

    print(f"Distance ssR : {dis_ssR} cm")
    print(f"Distance ssL : {dis_ssL} cm")
    print(io_data)
    

# --------------------------------------------------

def sub_attitude_handler(attitude_info):
    yaw, pitch, roll = attitude_info
    #yaw ดริฟรถ<z> pitch backflip ตีลังกา 3 ตะหลบ<y> roll  หมุนพวงมาลัย<x>
    print("chassis attitude: yaw:{0}, pitch:{1}, roll:{2} ".format(yaw, pitch, roll))
    global direction_facing
    current_yaw = yaw

    if -45 < current_yaw <= 45:
        direction_facing = 'N'

    elif 45 < current_yaw < 135:
        direction_facing = 'E'

    elif 135 < current_yaw < 180 or -180 < yaw < -135:
        direction_facing = 'S'

    elif -135 < current_yaw <= -45:
        direction_facing = 'W'
    print(direction_facing)


# --------------------------------------------------

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


# --------------------------------------------------

# def direction_now(yaw):
#     global direction_facing
#     current_yaw = yaw

#     if -45 < current_yaw <= 45:
#         direction_facing = 'N'

#     elif 45 < current_yaw < 135:
#         direction_facing = 'E'

#     elif 135 < current_yaw < 180 or -180 < yaw < -135:
#         direction_facing = 'S'

#     elif -135 < current_yaw <= -45:
#         direction_facing = 'W'

# ---------------------------------------------------

def update_position():
    global current_position

    x, y = current_position
    boundary_reached = False

    # Check boundary conditions for each direction
    if direction_facing == 'N' and y < 6:
        current_position = (x, y + 1)
    elif direction_facing == 'E' and x < 6:
        current_position = (x + 1, y)
    elif direction_facing == 'S' and y > 0:
        current_position = (x, y - 1)
    elif direction_facing == 'W' and x > 0:
        current_position = (x - 1, y)
    else:
        boundary_reached = True
        print(f"Boundary reached! Returning to the latest branch at {branches[-1] if branches else 'start'}.")

    if boundary_reached:
        # Return to the latest branch if the boundary is reached
        if branches:
            current_position = branches.pop()  # Move back to the last branch
        else:
            print("No branches to return to, staying at the current position.")

    visited_positions.append(current_position)
    print(f"Current Position: {current_position}")


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
                          # เซ็นเซอร์ซ้าย
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

# --------------------------------------------------

def adjust_left(): #sensor ir
    global err_dis_l
    if dis_ssL < 40:
        err_dis_l = (dis_ssL-23)/100
        if abs(err_dis_l) >= 0.01:
            return True
    return False

# --------------------------------------------------

def adjust_right(): #sensor ir
    global err_dis_r
    if dis_ssR < 40:
        err_dis_r = (dis_ssR-23)/100
        if abs(err_dis_r) >= 0.01:
            return True
    return False

# --------------------------------------------------

def adjust_front(): #sensor ir
    global err_dis_f
    err_dis_f = (tof_distance-200)/1000
    if abs(err_dis_f) >= 0.01:
        return True
    return False

# --------------------------------------------------

def adjust_pos():
    if front_wall():
        adjust_front()
    if check_wall_right():    
        adjust_right()
    if check_wall_left():
        adjust_left()


# --------------------------------------------------

# def turn_to(target_direction):
#     global current_direction
#     if current_direction == target_direction:
#         return
#     elif (current_direction, target_direction) in [('N', 'E'), ('E', 'S'), ('S', 'W'), ('W', 'N')]:
#         turn_right()
#     elif (current_direction, target_direction) in [('N', 'W'), ('W', 'S'), ('S', 'E'), ('E', 'N')]:
#         turn_left()
#     else:
#         turn_around()
#     current_direction = target_direction

# --------------------------------------------------

def check_terrorist():
    pass

# --------------------------------------------------

def move_back_to_branch():
    global current_position
    # Move back to the last branch point when no valid moves
    last_branch = branches.pop()
    current_position = last_branch
    print(f"Moving back to branch at {last_branch}")


# --------------------------------------------------



# --------------------------------------------------

def branch():
    if front_wall() and check_wall_left() and check_wall_right() == False:
        return True
    elif front_wall() and check_wall_left() == False:
        return True
    elif check_wall_right() and check_wall_left() == False:
        return True
    elif front_wall() and check_wall_right() == False:
        return True
    return False

# --------------------------------------------------

def process_movement():
    move_forward()

# --------------------------------------------------

def maze_explored():
    global current_position
    
    while True:
        if direction_facing == 'N':
            if branch():
                branches.append(current_position)
            if front_wall() or current_position in visited_positions:
                if check_wall_left() or current_position in visited_positions:
                    if check_wall_right() or current_position in visited_positions:
                        # adjust_pos()
                        turn_around()
                        move_back_to_branch()
                    else:
                        # adjust_pos()
                        turn_right()
                        process_movement()
                else:
                    # adjust_pos()
                    turn_left()
                    process_movement()
            else:
                # adjust_pos()
                process_movement()

        elif direction_facing == 'W':
            if branch():
                branches.append(current_position)
            if check_wall_right() or current_position in visited_positions:
                if front_wall() or current_position in visited_positions:
                    if check_wall_left() or current_position in visited_positions:
                        # adjust_pos()
                        turn_around()
                        move_back_to_branch()
                    else:
                        # adjust_pos()
                        turn_left()
                        process_movement()
                else:
                    turn_right()
                    # adjust_pos()
                    process_movement()
            else:
                turn_right()
                    # adjust_pos()
                process_movement()

        elif direction_facing == 'E':
            if branch():
                branches.append(current_position)
            if check_wall_left() or current_position in visited_positions:
                if front_wall() or current_position in visited_positions:
                    if check_wall_right() or current_position in visited_positions:
                        # adjust_pos()
                        turn_around()
                        move_back_to_branch()
                    else:
                        # adjust_pos()
                        turn_right()
                        process_movement()
                else:
                    # adjust_pos()
                    process_movement()
            else:
                turn_left()
                process_movement()

        elif direction_facing == 'S':
            if branch():
                branches.append(current_position)
            if check_wall_right() or current_position in visited_positions:
                if check_wall_left() or current_position in visited_positions:
                    if front_wall() or current_position in visited_positions:
                        # adjust_pos()
                        turn_around()
                        move_back_to_branch()
                    else:
                        # adjust_pos()
                        process_movement()
                else:
                    # adjust_pos()
                    turn_left()
                    process_movement()
            else:
                turn_right()
                process_movement()

# --------------------------------------------------

# def sub_position_handler(position_info):
#     x, y, z = position_info
#     print("chassis position: x:{0}, y:{1}, time:{2}".format(x, y, z))

# --------------------------------------------------

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis
    ep_chassis.sub_attitude(freq=10, callback=sub_attitude_handler)
    # ep_chassis.sub_position(freq=10, callback=sub_position_handler)

    ep_sensor = ep_robot.sensor_adaptor
    ep_sensor.sub_adapter(freq=10, callback=sub_data_handler)

    ep_tof = ep_robot.sensor
    ep_tof.sub_distance(freq=10, callback=tof_data_handler)

    ep_gimbal = ep_robot.gimbal
    ep_gimbal.recenter().wait_for_completed()
    time.sleep(3)

    try:
        maze_explored()  # Main exploration logic
    except KeyboardInterrupt:
        print("Process interrupted by user")
    finally:
        try:
            if ep_tof:
                ep_tof.unsub_distance()  # Unsubscribe only if ep_tof exists
            if ep_sensor:
                ep_sensor.unsub_adapter()  # Unsubscribe adapter only if ep_sensor exists
            ep_chassis.unsub_attitude()  # Unsubscribe from attitude data
        except AttributeError as e:
            print(f"Error during unsubscribing: {e}")
        ep_robot.close()  # Ensure the robot connection is closed
