import cv2
import robomaster
from robomaster import robot, camera
import time
import numpy as np

def bro_thinks_its_chicken():
    chicken_contour_max = max(chicken_contour, key=cv2.contourArea)
    chicken_x_axis, chicken_y_axis, chicken_width, chicken_height = cv2.boundingRect(chicken_contour_max)

    if chicken_width > 65:
        weight_factor = 0.3
        height_factor = 0.6
        y_offset = 20
    else:
        weight_factor = 0.3
        height_factor = 0.5
        y_offset = 5

    adjust_width = int(chicken_width * weight_factor)
    adjust_height = int(chicken_height * height_factor)
    new_chicken_x = chicken_x_axis - adjust_width // 2
    new_chicken_y = chicken_y_axis - adjust_height // 2 + y_offset
    new_chicken_width = chicken_width + adjust_width
    new_chicken_height = chicken_height + adjust_height
    cv2.rectangle(img, 
                  (new_chicken_x, new_chicken_y), 
                  (new_chicken_x + new_chicken_width, new_chicken_y + new_chicken_height), 
                  (0, 255, 0), 2)
    cv2.putText(img, "kukkai", 
                (chicken_x_axis, chicken_y_axis - 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

def bro_thinks_its_bottle(bottle_contour, img):
    bottle_contour_max = max(bottle_contour, key=cv2.contourArea)
    bottle_x, bottle_y, bottle_width, bottle_height = cv2.boundingRect(bottle_contour_max)

    if bottle_width > 85:   
        width_factor = 0.27
        height_factor = 2.5
        y_offset = -10
    elif bottle_width > 50:  
        width_factor = 0.27
        height_factor = 2.9
        y_offset = -11
    elif bottle_width > 35:  
        width_factor = 0.28
        height_factor = 3.4
        y_offset = -8
    else:
        width_factor = 0.47
        height_factor = 3.4
        y_offset = -4

    adjust_width = int(bottle_width * width_factor)
    adjust_height = int(bottle_height * height_factor)
    new_bottle_x = bottle_x - adjust_width // 2 
    new_bottle_y = bottle_y - adjust_height // 2 + y_offset
    new_bottle_width = bottle_width + adjust_width
    new_bottle_height = bottle_height + adjust_height

    cv2.rectangle(img, 
                  (new_bottle_x, new_bottle_y), 
                  (new_bottle_x + new_bottle_width, new_bottle_y + new_bottle_height), 
                  (0, 255, 0), 2)

    cv2.putText(img, "kukkuad", 
                (new_bottle_x, new_bottle_y - 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)


if __name__ == "__main__":
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_gimbal = ep_robot.gimbal
    ep_chassis = ep_robot.chassis

    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()
    ep_gimbal.moveto(pitch=-10, yaw=0).wait_for_completed()

    background = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
    background = cv2.cvtColor(background, cv2.COLOR_BGR2GRAY)
    background = cv2.GaussianBlur(background, (21, 21), 0)
    ep_chassis.move(x=1.2, y=0, z=0, xy_speed=0.1)
    
    while True:
        img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)

        gray_blurred = cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (21, 21), 0)

        diff = cv2.absdiff(background, gray_blurred)

        _,img_threshold = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)

        img_threshold = cv2.dilate(img_threshold, None, iterations=2)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_hue_chicken = np.array([33, 150, 100])
        upper_hue_chicken = np.array([36, 255, 255])
        lower_hue_bottle = np.array([95, 80, 100])
        upper_hue_bottle = np.array([120, 255, 255])

        chicken_mask = cv2.inRange(hsv, lower_hue_chicken, upper_hue_chicken)
        bottle_mask = cv2.inRange(hsv, lower_hue_bottle, upper_hue_bottle)

        chicken_segment = cv2.bitwise_and(img, img, mask=chicken_mask)
        bottle_segment = cv2.bitwise_and(img, img, mask=bottle_mask)

        chicken_contour, _ = cv2.findContours(
            chicken_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        bottle_contour, _ = cv2.findContours(
            bottle_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if chicken_contour: 
            bro_thinks_its_chicken()
        if bottle_contour:
            bro_thinks_its_bottle(bottle_contour, img)  # Pass the arguments here

        cv2.imshow("Bottle n Chicken", img)
        
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        time.sleep(0.1)

    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()
