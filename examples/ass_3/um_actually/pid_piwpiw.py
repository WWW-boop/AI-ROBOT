import cv2
import numpy as np
import time
from robomaster import robot, blaster, vision, camera
from scipy.spatial.distance import cosine

# PID control parameters
p = 0.2
i = 0.01
d = 0.05

# Smoothing factor for bounding box and gimbal speed
alpha = 0.7  # Smoothing factor for bounding box
max_speed = 200  # Maximum speed for gimbal movement

# Function to smooth bounding box coordinates or gimbal speed
def smooth_bbox(smooth_val, new_val, alpha):
    return smooth_val * (1 - alpha) + new_val * alpha

# Bottle detection function (blue cap detection)
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
        global bottle_contours_max
        bottle_contour_max = max(bottle_contours, key=cv2.contourArea)

        if 50 < cv2.contourArea(bottle_contour_max) < 5000:  # Minimum and maximum size
            approx = cv2.approxPolyDP(bottle_contour_max, 0.02 * cv2.arcLength(bottle_contour_max, True), True)
            if len(approx) > 4:  # Check the number of sides
                x, y, w, h = cv2.boundingRect(bottle_contour_max)
                aspect_ratio = float(w) / h
                if 0.8 < aspect_ratio < 1.2:  # Check aspect ratio
                    cv2.drawContours(img, [bottle_contour_max], -1, (0, 255, 0), 2)
                    cv2.putText(img, "Bottle Detected", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

                    return x, y, w, h  # Return bounding box coordinates
    return None

def main():
    # Initialize RoboMaster and gimbal
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_gimbal = ep_robot.gimbal
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    ep_gimbal.sub_angle(freq=10)
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()

    time.sleep(1)  # Wait for initialization

    detected = None

    # Initialize PID tracking variables
    prev_err_x, prev_err_y = 0, 0
    accumulate_err_x, accumulate_err_y = 0, 0
    prev_time = time.time()

    # Initialize smoothing variables for bounding box
    smooth_x, smooth_y, smooth_w, smooth_h = 0, 0, 0, 0

    # Initialize smoothed gimbal speeds
    smooth_speed_x, smooth_speed_y = 0, 0

    # Center of the camera view
    center_x = 1280 / 2
    center_y = 720 / 2

    while True:
        # Get the latest image from RoboMaster camera
        img = ep_camera.read_cv2_image(strategy="newest")
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Detect the bottle using blue cap detection
        bottle_bbox = blue_head_culprit(hsv, img)

        if bottle_bbox:
            after_time = time.time()
            x, y, w, h = bottle_bbox

            # Compute the center of the detected bottle
            center_x_detected = x + w / 2
            center_y_detected = y + h / 2

            # Calculate errors in X and Y axis
            err_x = center_x - center_x_detected
            err_y = center_y - center_y_detected
            accumulate_err_x += err_x * (after_time - prev_time)
            accumulate_err_y += err_y * (after_time - prev_time)

            # PID control for gimbal movement
            speed_x = p * err_x + i * accumulate_err_x + d * (err_x - prev_err_x)
            speed_y = p * err_y + i * accumulate_err_y + d * (err_y - prev_err_y)

            # Smooth the gimbal speed
            smooth_speed_x = smooth_bbox(smooth_speed_x, speed_x, alpha)
            smooth_speed_y = smooth_bbox(smooth_speed_y, speed_y, alpha)

            # Apply maximum speed limit
            smooth_speed_x = max(min(smooth_speed_x, max_speed), -max_speed)
            smooth_speed_y = max(min(smooth_speed_y, max_speed), -max_speed)

            # Move the gimbal based on the smoothed speeds
            ep_gimbal.drive_speed(pitch_speed=smooth_speed_y, yaw_speed=-smooth_speed_x)

            # Update previous errors and time
            prev_time = after_time
            prev_err_x = err_x
            prev_err_y = err_y

            # Smoothing bounding box for display
            smooth_x = smooth_bbox(smooth_x, center_x_detected, alpha)
            smooth_y = smooth_bbox(smooth_y, center_y_detected, alpha)
            smooth_w = smooth_bbox(smooth_w, w, alpha)
            smooth_h = smooth_bbox(smooth_h, h, alpha)

        else:
            # If no target is detected, stop gimbal movement
            ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)

        # Draw crosshair at the center of the screen
        crosshair_size = 10  # Length of each line in pixels
        crosshair_color = (255, 255, 255)  # Red crosshair
        crosshair_thickness = 1  # Line thickness

        # Horizontal line
        cv2.line(img, (int(center_x - crosshair_size), int(center_y)), (int(center_x + crosshair_size), int(center_y)), crosshair_color, crosshair_thickness)
        # Vertical line
        cv2.line(img, (int(center_x), int(center_y - crosshair_size)), (int(center_x), int(center_y + crosshair_size)), crosshair_color, crosshair_thickness)

        # Display the result in a window
        cv2.imshow("RoboMaster Detection", img)
   
        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Clean up
    ep_camera.stop_video_stream()
    ep_robot.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
