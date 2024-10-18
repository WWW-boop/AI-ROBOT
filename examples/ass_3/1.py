import cv2
import numpy as np
import time
from robomaster import robot, blaster, camera

# Global variables
tof_distance = None
io_data = None
dis_ssL = None
dis_ssR = None
ir_left = None
ir_right = None
yaw = None
action_state = None
now_pos = (1, 3)
direction_facing = 'N'

# PID control parameters
p = 0.2
i = 0.01
d = 0.05

# Smoothing factor for bounding box and gimbal speed
alpha = 0.7
max_speed = 200

# Function definitions from the first script
def update_position_on_move():
    global now_pos
    if direction_facing == 'N':
        now_pos = (now_pos[0] + 1, now_pos[1])
    elif direction_facing == 'E':
        now_pos = (now_pos[0], now_pos[1] + 1)
    elif direction_facing == 'S':
        now_pos = (now_pos[0] - 1, now_pos[1])
    elif direction_facing == 'W':
        now_pos = (now_pos[0], now_pos[1] - 1)
    print(f"New Position: {now_pos}")

def move_forward():
    ep_robot.chassis.move(x=0.6, y=0, z=0, xy_speed=0.7).wait_for_completed()
    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()
    print("------- move forward -------")
    update_position_on_move()

def turn_left():
    global direction_facing
    ep_robot.chassis.move(x=0, y=0, z=90, z_speed=100).wait_for_completed()
    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()
    direction_facing = {'N': 'W', 'W': 'S', 'S': 'E', 'E': 'N'}[direction_facing]
    print("------- turn left -------")
    print(f"Now facing: {direction_facing}")

def turn_right():
    global direction_facing
    ep_robot.chassis.move(x=0, y=0, z=-91, z_speed=100).wait_for_completed()
    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()
    direction_facing = {'N': 'E', 'E': 'S', 'S': 'W', 'W': 'N'}[direction_facing]
    print("------- turn right -------")
    print(f"Now facing: {direction_facing}")

def turn_around():
    global direction_facing
    ep_robot.chassis.move(x=0, y=0, z=180, z_speed=100).wait_for_completed()
    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()
    direction_facing = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}[direction_facing]
    print("------- turn around -------")
    print(f"Now facing: {direction_facing}")

def stop_moving():
    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    time.sleep(0.5)

# Include other functions from the first script (tof_data_handler, filter_ad_data, sub_data_handler, etc.)
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
    if direction_facing == 'N':
        if now_pos[0] == 6:
            print(f"Wall detected at distance: {tof_distance}")
            return True
    elif direction_facing == 'W':
        if now_pos[1] == 1:
            print(f"Wall detected at distance: {tof_distance}")
            return True
    elif direction_facing == 'E':
        if now_pos[1] == 6:
            print(f"Wall detected at distance: {tof_distance}")
            return True
    elif direction_facing == 'S':
        if now_pos[0] == 1:
            print(f"Wall detected at distance: {tof_distance}")
            return True
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
    if direction_facing == 'N':
        if now_pos[1] == 6:
            print('Wall right')
            return True
    elif direction_facing == 'W':
        if now_pos[0] == 6:
            print('Wall right')
            return True
    elif direction_facing == 'E':
        if now_pos[0] == 1:
            print('Wall right')
            return True
    elif direction_facing == 'S':
        if now_pos[1] == 1:
            print('Wall right')
            return True
    # print('Right wall detected due to boundary condition (x or y = 6)')
    
    if ir_right == 0:
        print('Wall right')
        return True
    return False

# --------------------------------------------------
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
    if abs(dis_ssL - dis_ssR) >= 2:
        if dis_ssL > dis_ssR:
            move = ((dis_ssL - dis_ssR) /2)/100
            ep_robot.chassis.move(x=0, y=move, z=0, xy_speed=3).wait_for_completed()
        else:
            move = ((dis_ssR - dis_ssL) /2)/100
            ep_robot.chassis.move(x=0, y=-move, z=0, xy_speed=3).wait_for_completed()
        


#---------------------------------------------------

def adjust_front(): #sensor ir
    global err_dis_f
    err_dis_f = (tof_distance-200)/1000
    if tof_distance < 400:
        if abs(err_dis_f) >= 0.01:
            move = err_dis_f
            ep_robot.chassis.move(x=move, y=0, z=0, xy_speed=5).wait_for_completed()

# --------------------------------------------------

def adjust_pos():
    if front_wall() == True:
        adjust_front()
    if check_wall_right() == True and  check_wall_left() == False:    
        adjust_right()
    if check_wall_left() == True and  check_wall_right() == False:    
        adjust_left()
    if check_wall_left() == True and check_wall_right():
        adjust_position_LR()

# Functions from the second script
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

        if 50 < cv2.contourArea(bottle_contour_max) < 5000:
            approx = cv2.approxPolyDP(bottle_contour_max, 0.02 * cv2.arcLength(bottle_contour_max, True), True)
            if len(approx) > 4:
                x, y, w, h = cv2.boundingRect(bottle_contour_max)
                aspect_ratio = float(w) / h
                if 0.8 < aspect_ratio < 1.2:
                    cv2.drawContours(img, [bottle_contour_max], -1, (0, 255, 0), 2)
                    cv2.putText(img, "Bottle Detected", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                    return x, y, w, h
    return None

def body_acrylic_detect(img):
    template = cv2.imread("C:\\Users\\wikra\\Desktop\\robot ai\\RoboMaster-SDK\\examples\\ass_3\\jjjjjjj.jpg", 0)
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

# Combined main function
def main():
    global ep_robot, ep_chassis, ep_gimbal, ep_blaster, ep_camera, ep_sensor, ep_tof

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_gimbal = ep_robot.gimbal
    ep_blaster = ep_robot.blaster
    ep_camera = ep_robot.camera
    ep_sensor = ep_robot.sensor_adaptor
    ep_tof = ep_robot.sensor

    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    ep_gimbal.sub_angle(freq=10)
    ep_chassis.sub_attitude(freq=20, callback=sub_attitude_handler)
    ep_sensor.sub_adapter(freq=20, callback=sub_data_handler)
    ep_tof.sub_distance(freq=20, callback=tof_data_handler)

    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()

    time.sleep(1)

    detected = None
    detection_start_time = None
    detection_duration = 0

    center_x = 1280 / 2
    center_y = 720 / 2

    try:
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
                    ep_blaster.fire(fire_type=blaster.WATER_FIRE, times=5)
                    time.sleep(2)
            else:
                detection_start_time = None
                detection_duration = 0

                # Execute maze navigation logic when no bottle is detected
                if front_wall() == True and check_wall_right() == True and check_wall_left() == True:
                    stop_moving()
                    turn_around()
                    adjust_pos()
                    move_forward()
                    adjust_pos()
                    action_state = 'around'
                elif front_wall() == False and check_wall_right() == True:
                    stop_moving()
                    move_forward()
                    adjust_pos()
                    action_state = 'forward'
                elif check_wall_right() == False:            
                    stop_moving()
                    turn_right()
                    adjust_pos()
                    move_forward()
                    adjust_pos()
                    action_state = 'right'
                elif front_wall() == True and check_wall_right() == True:
                    stop_moving()
                    turn_left()
                    adjust_pos()
                    move_forward()
                    adjust_pos()
                    action_state = 'turn left'

            img = body_acrylic_detect(img)
            img = gai_detect(hsv, img)

            cv2.imshow("Detection", img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Process interrupted by user")
    finally:
        ep_camera.stop_video_stream()
        ep_tof.unsub_distance()
        ep_sensor.unsub_adapter()
        ep_chassis.unsub_attitude()
        ep_robot.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()