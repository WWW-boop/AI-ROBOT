from robomaster import robot
import time
import cv2
import numpy as np
import time
from robomaster import robot, blaster, camera


current_position = (3, 3)  # Starting at position (3, 3)
visited_positions = []
branches = []
criminals = []
p = 0.2
i = 0.01
d = 0.05

alpha = 0.7  # Smoothing factor for bounding box
max_speed = 200

def smooth_bbox(smooth_val, new_val, alpha):
    return smooth_val * (1 - alpha) + new_val * alpha

def blue_head_culprit(hsv, img):
    lower_hue_bottle = np.array([99, 122, 88])
    upper_hue_bottle = np.array([111, 246, 255])

    bottle_mask = cv2.inRange(hsv, lower_hue_bottle, upper_hue_bottle)

    kernel = np.ones((5, 5), np.uint8)
    bottle_mask = cv2.morphologyEx(bottle_mask, cv2.MORPH_CLOSE, kernel)
    bottle_mask = cv2.morphologyEx(bottle_mask, cv2.MORPH_OPEN, kernel)
    bottle_mask = cv2.GaussianBlur(bottle_mask, (5, 5), 0)

    bottle_contours, _ = cv2.findContours(bottle_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(bottle_contours) > 0:
        bottle_contour_max = max(bottle_contours, key=cv2.contourArea)

        if 50 < cv2.contourArea(bottle_contour_max) < 5000:  # Minimum and maximum size
            approx = cv2.approxPolyDP(bottle_contour_max, 0.02 * cv2.arcLength(bottle_contour_max, True), True)
            if len(approx) > 4:  # Check the number of sidesq
                x, y, w, h = cv2.boundingRect(bottle_contour_max)
                aspect_ratio = float(w) / h
                if 0.8 < aspect_ratio < 1.2:  # Check aspect ratio
                    cv2.drawContours(img, [bottle_contour_max], -1, (0, 255, 0), 2)
                    cv2.putText(img, "Bottle Detected", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

                    return x, y, w, h  # Return bounding box coordinates
    return None


def body_acrylic_detect(img):
    template = cv2.imread(r"C:\Users\lataeq\AI-ROBOT\jjjjjjj.jpg", 0)
    if template is None:
        print("Template not found!")
        return None

    mask = np.zeros(img.shape[:2], dtype="uint8")
    cv2.rectangle(mask, (0, 0), (1280, 340), 255, -1)
    blurred_image = cv2.GaussianBlur(img, (99, 99), 0)
    result = np.where(mask[:, :, np.newaxis] == 255, blurred_image, img)

    gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    edges = cv2.Canny(blurred, 65, 225)
    template_blurred = cv2.GaussianBlur(template, (5, 5), 0)
    template_edges = cv2.Canny(template_blurred, 1, 1)
    
    res = cv2.matchTemplate(edges, template_edges, cv2.TM_CCOEFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

    if max_val > 0.5:
        height, width = template.shape
        cv2.rectangle(img, max_loc, (max_loc[0] + width, max_loc[1] + height), (0, 255, 0), 2)
        cv2.putText(img, "Acrylic Detected", (max_loc[0], max_loc[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

    return img 


def gai_detect(hsv, img):
    x_start, y_start, x_end, y_end = 320, 320, 960, 670  
    roi = img[y_start:y_end, x_start:x_end]
    hsv_roi = hsv[y_start:y_end, x_start:x_end]

    lower_hue_gai = np.array([29, 235, 85])
    upper_hue_gai = np.array([37, 255, 255])
    gai_mask = cv2.inRange(hsv_roi, lower_hue_gai, upper_hue_gai)

    kernel = np.ones((5, 5), np.uint8)
    gai_mask = cv2.GaussianBlur(gai_mask, (9, 9), 0)
    gai_mask = cv2.morphologyEx(gai_mask, cv2.MORPH_CLOSE, kernel)
    gai_mask = cv2.morphologyEx(gai_mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(gai_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 100:  
            x, y, w, h = cv2.boundingRect(cnt)
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)

            if len(approx) > 5: 
                cv2.drawContours(roi, [approx], -1, (0, 255, 0), 3)
                cv2.putText(roi, "Chicken Detected", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

    return img

def main():
    time.sleep(1)  

    detected = None
    detection_start_time = None  
    detection_duration = 0  

    center_x = 1280 / 2
    center_y = 720 / 2

    while True:
        img = ep_camera.read_cv2_image(strategy="newest")
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        bottle_bbox = blue_head_culprit(hsv, img)

        if bottle_bbox:
            x, y, w, h = bottle_bbox

            error_x = (x + w / 2) - center_x
            error_y = (y + h / 2) - center_y

            yaw_speed = p * error_x
            pitch_speed = p * error_y

            yaw_speed = np.clip(yaw_speed, -max_speed, max_speed)
            pitch_speed = np.clip(pitch_speed, -max_speed, max_speed)

            ep_gimbal.drive_speed(pitch_speed=-pitch_speed, yaw_speed=yaw_speed)

            if detection_start_time is None:
                detection_start_time = time.time()  
            detection_duration = time.time() - detection_start_time  

            if detection_duration > 3:
                cv2.putText(img, "Firing!", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                check_terrorist(current_position)
        else:
            detection_start_time = None
            detection_duration = 0

            # Recenter the gimbal when no bottle is detected
            ep_gimbal.recenter(pitch_speed=100, yaw_speed=100).wait_for_completed()

        img = body_acrylic_detect(img)
        img = gai_detect(hsv, img)

        cv2.imshow("Detection", img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# --------------------------------------------------

def move_forward():
    
    ep_robot.chassis.move(x=0.6, y=0, z=0, xy_speed=0.5).wait_for_completed()
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
    # print(f"TOF distance: {tof_distance}")
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

    # print(f"Distance ssR : {dis_ssR} cm")
    # print(f"Distance ssL : {dis_ssL} cm")
    print(io_data)
    

# --------------------------------------------------

def sub_attitude_handler(attitude_info):
    yaw, pitch, roll = attitude_info
    #yaw ดริฟรถ<z> pitch backflip ตีลังกา 3 ตะหลบ<y> roll  หมุนพวงมาลัย<x>
    # print("chassis attitude: yaw:{0}, pitch:{1}, roll:{2} ".format(yaw, pitch, roll))
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
    err_dis_l = (dis_ssL-23)/100
    if abs(err_dis_l) >= 0.01:
        ep_chassis.move(x=0, y=err_dis_l/100, z=0, xy_speed=0.5).wait_for_completed()

# --------------------------------------------------

def adjust_right(): #sensor ir
    global err_dis_r
    err_dis_r = (dis_ssR-23)/100
    if abs(err_dis_r) >= 0.01:
        ep_chassis.move(x=0, y=err_dis_r/100, z=0, xy_speed=0.5).wait_for_completed()
# --------------------------------------------------
def adjust_position_LR():
    if abs(dis_ssL - dis_ssR) >= 0.1:
        if dis_ssL > dis_ssR:
            move = (dis_ssL - dis_ssR) / 2 /100
            ep_robot.chassis.move(x=0, y=move, z=0, xy_speed=0.5).wait_for_completed()
        else:
            move = (dis_ssR - dis_ssL) / 2/100
            ep_robot.chassis.move(x=0, y=-move, z=0, xy_speed=0.5).wait_for_completed()
        


#---------------------------------------------------

def adjust_front(): #sensor ir
    global err_dis_f
    err_dis_f = (tof_distance-200)/1000
    if abs(err_dis_f) >= 0.01:
        move = err_dis_f
        ep_robot.chassis.move(x=move/1000, y=0, z=0, xy_speed=0.5).wait_for_completed()

# --------------------------------------------------

def adjust_pos():
    if front_wall():
        adjust_front()
    if check_wall_right() and not check_wall_left():    
        adjust_right()
    if check_wall_left() and not check_wall_right():    
        adjust_left()
    if check_wall_left() and check_wall_right():
        adjust_position_LR()


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
    print(f"Terrorist detected and shot at position: {position}")

# --------------------------------------------------

# def move_back_to_branch():
#     global current_position
#     # Move back to the last branch point when no valid moves
#     last_branch = branches.pop()
#     current_position = last_branch
#     print(f"Moving back to branch at {last_branch}")


# --------------------------------------------------

def dead_end():
    if front_wall() and check_wall_left() and check_wall_right() == True:
        return True
    else:
        return False

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
#     update_position()  # Update position after moving
#     print(f"Current Position: {current_position}")  # Print updated position

# # --------------------------------------------------

def maze_explored():
    global current_position
    while True:
        print('maze_explored')
        print(visited_positions)
        time.sleep(0.1)
        x, y = current_position
        if direction_facing == 'N':
            # if branch():
            #     branches.append(current_position)
            if front_wall() or (x, y+1) in visited_positions:
                if check_wall_left() or (x-1, y) in visited_positions:
                    if check_wall_right() or (x+1, y) in visited_positions:
                        adjust_pos()
                        turn_around()
                        maze_reverse_explored()
                        
                    else:
                        adjust_pos()
                        turn_right()
                        process_movement()
                        update_position()  # Update position after moving
                        print(f"Current Position: {current_position}")
                else:
                    adjust_pos()
                    turn_left()
                    process_movement()
                    update_position()  # Update position after moving
                    print(f"Current Position: {current_position}")
            else:
                adjust_pos()
                process_movement()
                update_position()  # Update position after moving
                print(f"Current Position: {current_position}")

        elif direction_facing == 'W':
            # if branch():
            #     branches.append(current_position)
            if check_wall_right() or (x, y+1) in visited_positions:
                if front_wall() or (x-1, y) in visited_positions:
                    if check_wall_left() or (x, y-1) in visited_positions:
                        adjust_pos()
                        turn_around()
                        # move_back_to_branch()
                        maze_reverse_explored()  # Update position after moving
                        print(f"Current Position: {current_position}")
                    else:
                        adjust_pos()
                        turn_left()
                        process_movement()
                        update_position()  # Update position after moving
                        print(f"Current Position: {current_position}")
                else:
                    turn_right()
                    adjust_pos()
                    process_movement()
                    update_position()  # Update position after moving
                    print(f"Current Position: {current_position}")
            else:
                turn_right()
                adjust_pos()
                process_movement()
                update_position()  # Update position after moving
                print(f"Current Position: {current_position}")

        elif direction_facing == 'E':
            # if branch():
            #     branches.append(current_position)
            if check_wall_left() or (x, y+1) in visited_positions:
                if front_wall() or (x+1, y) in visited_positions:
                    if check_wall_right() or (x, y-1) in visited_positions:
                        adjust_pos()
                        turn_around()
                        # move_back_to_branch()
                        maze_reverse_explored()  # Update position after moving
                        print(f"Current Position: {current_position}")
                    else:
                        adjust_pos()
                        turn_right()
                        process_movement()
                        update_position()  # Update position after moving
                        print(f"Current Position: {current_position}")
                else:
                    adjust_pos()
                    process_movement()
                    update_position()  # Update position after moving
                    print(f"Current Position: {current_position}")
            else:
                adjust_pos()
                turn_left()
                process_movement()
                update_position()  # Update position after moving
                print(f"Current Position: {current_position}")

        elif direction_facing == 'S':
            # if branch():
            #     branches.append(current_position)
            if check_wall_right() or (x-1, y) in visited_positions:
                if check_wall_left() or (x+1, y) in visited_positions:
                    if front_wall() or (x, y-1) in visited_positions:
                        adjust_pos()
                        turn_around()
                        # move_back_to_branch()
                        maze_reverse_explored()  # Update position after moving
                        print(f"Current Position: {current_position}")
                    else:
                        adjust_pos()
                        process_movement()
                        update_position()  # Update position after moving
                        print(f"Current Position: {current_position}")
                else:
                    adjust_pos()
                    turn_left()
                    process_movement()
                    update_position()  # Update position after moving
                    print(f"Current Position: {current_position}")
            else:
                adjust_pos()
                turn_right()
                process_movement()
                update_position()  # Update position after moving
                print(f"Current Position: {current_position}")

def maze_reverse_explored():
    global current_position
    while True:
        print('maze_reverse_explored')
        print(visited_positions)
        time.sleep(0.1)
        x, y = current_position
        if direction_facing == 'S':
            #     branches.append(current_position)
            if front_wall() or (x, y-1) not in visited_positions:
                if check_wall_left() or (x+1, y) not in visited_positions:
                    if check_wall_right() or (x-1, y) not in visited_positions:
                        adjust_pos()
                        if branch():
                            maze_explored()# Update position after moving
                            print(f"Current Position: {current_position}")
                        # move_back_to_branch()
                    else:
                        adjust_pos()
                        turn_right()
                        process_movement()
                        update_position()  # Update position after moving
                        print(f"Current Position: {current_position}")
                else:
                    adjust_pos()
                    turn_left()
                    process_movement()
                    update_position()  # Update position after moving
                    print(f"Current Position: {current_position}")
            else:
                adjust_pos()
                process_movement()
                update_position()  # Update position after moving
                print(f"Current Position: {current_position}")

        elif direction_facing == 'E':
            if check_wall_right() or (x, y-1) not in visited_positions:
                if front_wall() or (x+1, y) not in visited_positions:
                    if check_wall_left() or (x, y+1) not in visited_positions:
                        if branch():
                            maze_explored()
                        # move_back_to_branch()
                        # Update position after moving
                            print(f"Current Position: {current_position}")
                    else:
                        adjust_pos()
                        turn_left()
                        process_movement()
                        update_position()  # Update position after moving
                        print(f"Current Position: {current_position}")
                else:
                    adjust_pos()
                    process_movement()
                    update_position()  # Update position after moving
                    print(f"Current Position: {current_position}")
            else:
                turn_right()
                adjust_pos()
                process_movement()
                update_position()  # Update position after moving
                print(f"Current Position: {current_position}")

        elif direction_facing == 'W':
            # if branch():
            #     branches.append(current_position)
            if check_wall_left() or (x, y-1) not in visited_positions:
                if front_wall() or (x-1, y) not in visited_positions:
                    if check_wall_right() or (x, y+1) not in visited_positions:
                        if branch():
                            maze_explored()  # Update position after moving
                            print(f"Current Position: {current_position}")
                    else:
                        adjust_pos()
                        turn_right()
                        process_movement()
                        update_position()  # Update position after moving
                        print(f"Current Position: {current_position}")
                else:
                    adjust_pos()
                    process_movement()
                    update_position()  # Update position after moving
                    print(f"Current Position: {current_position}")
            else:
                turn_left()
                process_movement()
                update_position()  # Update position after moving
                print(f"Current Position: {current_position}")

        elif direction_facing == 'N':
            # if branch():
            if branch():
                maze_explored()
            #     branches.append(current_position)
            if check_wall_right() or (x+1, y) not in visited_positions:
                if check_wall_left() or (x-1, y) not in visited_positions:
                    if front_wall() or (x, y+1) not in visited_positions:
                        if branch():
                            maze_explored()  # Update position after moving
                            print(f"Current Position: {current_position}")
                    else:
                        adjust_pos()
                        process_movement()
                        update_position()  # Update position after moving
                        print(f"Current Position: {current_position}")
                else:
                    adjust_pos()
                    turn_left()
                    process_movement()
                    update_position()  # Update position after moving
                    print(f"Current Position: {current_position}")
            else:
                adjust_pos()
                turn_right()
                process_movement()
                update_position()  # Update position after moving
                print(f"Current Position: {current_position}")

# --------------------------------------------------

# def sub_position_handler(position_info):
#     x, y, z = position_info
#     print("chassis position: x:{0}, y:{1}, time:{2}".format(x, y, z))

# --------------------------------------------------

if __name__ == '__main__':
    main()
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_gimbal = ep_robot.gimbal
    ep_blaster = ep_robot.blaster
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    ep_gimbal.sub_angle(freq=10)
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
            ep_chassis.unsub_attitude()
            ep_camera.stop_video_stream()
            ep_robot.close()
            cv2.destroyAllWindows()  # Unsubscribe from attitude data
        except AttributeError as e:
            print(f"Error during unsubscribing: {e}")
        ep_robot.close()  # Ensure the robot connection is closed
