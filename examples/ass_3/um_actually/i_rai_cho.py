from robomaster import robot
import time
import matplotlib.pyplot as plt
import numpy as np

# เริ่มต้นหุ่นยนต์ RoboMaster
ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")

current_direction = 'N'

# --------------------------------------------------

def move_forward():
    print("------- move forward -------")
    for i in range(2):
        ep_robot.chassis.move(x=0.3, y=0, z=0, xy_speed=10).wait_for_completed()

# --------------------------------------------------

def turn_left():
    print("------- turn left -------")
    ep_robot.chassis.move(x=0, y=0, z=90, z_speed=100).wait_for_completed()
    time.sleep(0.1)

# --------------------------------------------------

def turn_right():
    print("------- turn right -------")
    ep_robot.chassis.move(x=0, y=0, z=-91, z_speed=100).wait_for_completed()
    time.sleep(0.1)

# --------------------------------------------------
    
def turn_around():
    print("------- turn around -------")
    ep_robot.chassis.move(x=0, y=0, z=180, z_speed=100).wait_for_completed()
    time.sleep(0.1)

# --------------------------------------------------

def tof_data_handler(sub_info):
    global tof_distance
    tof_distance = sub_info[0]
    print("ToF: {0}".format(sub_info[0]))

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
    global io_data, dis_ssL, dis_ssR

    io_data, ad_data = sub_info
    smoothed_values = filter_ad_data(ad_data)

    ssR = smoothed_values[1] 
    ssL = smoothed_values[0] 
    vaR, vaL = convert_to_V(ssR, ssL)

    dis_ssR = convert_to_cm(vaR) / 2
    dis_ssL = convert_to_cm(vaL) / 2 

    print(f"Distance ssR : {dis_ssR} cm")
    print(f"Distance ssL : {dis_ssL} cm")

# --------------------------------------------------

def sub_attitude_handler(attitude_info):
    global yaw
    yaw, pitch, roll = attitude_info
    print("chassis attitude: yaw:{0}, pitch:{1}, roll:{2} ".format(yaw, pitch, roll))

# --------------------------------------------------

def yaw_ming(yaw):
    target_direction = 0
    current_yaw = yaw

    if -45 < yaw <= 45:
        target_direction = 0
        ep_chassis.move(x=0, y=0, z=current_yaw, z_speed=100).wait_for_completed()

    elif 45 < yaw < 135:
        target_direction = 90
        ep_chassis.move(x=0, y=0, z=current_yaw-target_direction, z_speed=100).wait_for_completed()

    elif 135 < yaw < 180 or -180 < yaw < -135:
        target_direction = 180
        ep_chassis.move(x=0, y=0, z=current_yaw-target_direction, z_speed=100).wait_for_completed() 

    elif -135 < yaw <= -45:
        target_direction = -90
        ep_chassis.move(x=0, y=0, z=current_yaw-target_direction, z_speed=100).wait_for_completed()  


# --------------------------------------------------

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
    if tof_distance is not None:
        if tof_distance <= 250:
            return True
    return False

# --------------------------------------------------

def check_wall_left(io_data): #sensor ir
    ir_left = io_data[3]  # เซ็นเซอร์ซ้าย
    if ir_left == 0:
        return True
    else:
        return False

# --------------------------------------------------

def check_wall_right(io_data): #sensor ir
    ir_right = io_data[2]  # เซ็นเซอร์ขวา
    if ir_right == 0:
        return True
    else:
        return False

# --------------------------------------------------

def update_maze_display():
    maze_image.set_data(maze_map)
    plt.draw()
    plt.pause(0.1)

# --------------------------------------------------

def turn_to(target_direction):
    global current_direction
    if current_direction == target_direction:
        return
    elif (current_direction, target_direction) in [('N', 'E'), ('E', 'S'), ('S', 'W'), ('W', 'N')]:
        turn_right()
    elif (current_direction, target_direction) in [('N', 'W'), ('W', 'S'), ('S', 'E'), ('E', 'N')]:
        turn_left()
    else:
        turn_around()
    current_direction = target_direction

# --------------------------------------------------

def dfs_solve():
    global stack, current_direction, io_data  # Add io_data here
    while stack:
        x, y = stack.pop()

        # Mark the current cell as visited (walkable path)
        maze_map[x, y] = 0
        update_maze_display()

        # Align robot's direction
        yaw_ming(yaw)

        # Check for a wall in front
        if front_wall():
            print("Wall in front")

            # Check for a wall on the left
            if check_wall_left(io_data):  # Pass io_data here
                print("Wall on left")
                # If there's a wall on the left, check for a wall on the right
                if check_wall_right(io_data):  # Pass io_data here
                    print("Wall on right too, turning around")
                    turn_around()  # Turn around if there's a wall on both sides
                else:
                    print("No wall on right, turning right")
                    turn_right()  # Turn right if no wall on the right
                    move_forward()  # Move forward after turning
            else:
                print("No wall on left, turning left")
                turn_left()  # Turn left if no wall on the left
                move_forward()  # Move forward after turning

        else:
            print("No wall in front, moving forward")
            move_forward()  # Move forward if there's no wall in front

        # Add a slight delay to simulate robot's movement
        time.sleep(0.1)

        # Explore in N, E, S, W priority order for the next cells
        for direction in direction_priority:
            dx, dy = direction_map[direction]
            nx, ny = x + dx, y + dy

            # Ensure the next move is within bounds
            if 0 <= nx < maze_size[0] and 0 <= ny < maze_size[1]:
                # Use sensors to detect walls in real-time
                if maze_map[nx, ny] == 1 and not front_wall():  # Unvisited and no obstacle
                    stack.append((nx, ny))
                    turn_to(direction)
                    move_forward()
                    time.sleep(0.1)  # Add a small delay to simulate the robot's movement
                    break

    # Check if exploration is complete
    if not stack:
        print("Exploration complete.")

# --------------------------------------------------

def sub_position_handler(position_info):
    x, y, z = position_info
    print("chassis position: x:{0}, y:{1}, time:{2}".format(x, y, z))

# --------------------------------------------------

if __name__ == '__main__':

    maze_size = (6, 6)  # 6x6 maze
    maze_map = np.ones(maze_size)  # Initially, mark everything as obstacles (1)

    # Robot's initial position
    start_x, start_y = 0, 0
    maze_map[start_x, start_y] = 0  # Mark start position as walkable (0)

    # Stack for DFS exploration
    stack = [(start_x, start_y)]

    # Current direction of the robot (initially facing North)
    current_direction = 'N'

    # Matplotlib for real-time visualization
    plt.ion()
    fig, ax = plt.subplots()
    maze_image = ax.imshow(maze_map, cmap='gray')
    plt.show()

    # Directions for DFS with priorities N, E, S, W
    direction_map = {
        'N': (-1, 0),
        'E': (0, 1),
        'S': (1, 0),
        'W': (0, -1)
    }
    direction_priority = ['N', 'E', 'S', 'W']  # N, E, S, W priority

    ep_chassis = ep_robot.chassis
    ep_chassis.sub_attitude(freq=10, callback=sub_attitude_handler)
    ep_chassis.sub_position(freq=10, callback=sub_position_handler)

    ep_sensor = ep_robot.sensor_adaptor
    ep_sensor.sub_adapter(freq=10, callback=sub_data_handler)
    ep_tof = ep_robot.sensor
    ep_tof.sub_distance(freq=10, callback=tof_data_handler)

    ep_gimbal = ep_robot.gimbal
    ep_gimbal.recenter().wait_for_completed()

    try:
        dfs_solve()
    except KeyboardInterrupt:
        print("Process interrupted")
    finally:
        ep_sensor.unsub_adapter()
        ep_tof.unsub_distance()  # Correct method name
        ep_chassis.unsub_position()
        ep_robot.close()