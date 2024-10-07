import cv2
import numpy as np
import time
from robomaster import robot, blaster, camera

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

# Template matching for acrylic detection
def body_acrylic_detect(img):
    # Load the template image in grayscale
    template = cv2.imread("C:\\Users\\User\\Documents\\GitHub\\AI-ROBOT\\jjjjjjj.jpg", 0)
    if template is None:
        print("Template not found!")
        return None

    # Convert input image to grayscale
    mask = np.zeros(img.shape[:2], dtype="uint8")
    cv2.rectangle(mask, (0, 0), (1280, 340), 255, -1)
    blurred_image = cv2.GaussianBlur(img, (99, 99), 0)
    result = np.where(mask[:, :, np.newaxis] == 255, blurred_image, img)

    gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Detect edges in the image
    edges = cv2.Canny(blurred, 65, 225)

    # Load and process template
    template_blurred = cv2.GaussianBlur(template, (5, 5), 0)
    template_edges = cv2.Canny(template_blurred, 1, 1)
    
    # Match template to the contours found in the image
    res = cv2.matchTemplate(edges, template_edges, cv2.TM_CCOEFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

    # Threshold for template matching
    if max_val > 0.5:
        height, width = template.shape
        # Draw rectangle around detected region
        cv2.rectangle(img, max_loc, (max_loc[0] + width, max_loc[1] + height), (0, 255, 0), 2)
        cv2.putText(img, "Acrylic Detected", (max_loc[0], max_loc[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

    # Display or return the resulting image
    return img 

def main():
    # Initialize RoboMaster and gimbal
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_gimbal = ep_robot.gimbal
    ep_blaster = ep_robot.blaster  # Initialize the blaster
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    ep_gimbal.sub_angle(freq=10)
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()

    time.sleep(1)  # Wait for initialization

    detected = None
    detection_start_time = None  # Initialize detection start time
    detection_duration = 0  # Duration of detection

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
            x, y, w, h = bottle_bbox

            # Calculate the error between the center of the object and the center of the screen
            error_x = (x + w / 2) - center_x
            error_y = (y + h / 2) - center_y

            # Use PID control to adjust gimbal movement
            yaw_speed = p * error_x
            pitch_speed = p * error_y

            # Ensure the speed does not exceed max_speed
            yaw_speed = np.clip(yaw_speed, -max_speed, max_speed)
            pitch_speed = np.clip(pitch_speed, -max_speed, max_speed)

            # Move the gimbal towards the object
            ep_gimbal.drive_speed(pitch_speed=-pitch_speed, yaw_speed=yaw_speed)

            if detection_start_time is None:
                detection_start_time = time.time()  # Start the timer
            detection_duration = time.time() - detection_start_time

            # พิมพ์เวลาในการตรวจจับ
            print(f"Detection duration: {detection_duration}")

            if detection_duration >= 5:  # Check if target is detected for at least 5 seconds
                body_acrylic_detect(img)  # Additional detection using template matching
                print("Target detected for 5 seconds! Firing!")
                #ep_blaster.fire(frequency=1)  # Fire once
        else:
            detection_start_time = None  # Reset the detection timer
            detection_duration = 0  # Reset detection duration
            # If no target is detected, stop gimbal movement
            ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)


        # เพิ่มการตรวจจับ acrylic body ตลอดเวลาทุกครั้งในลูป
        body_acrylic_detect(img)

        # Draw crosshair at the center of the screen
        cv2.line(img, (int(center_x - 10), int(center_y)), (int(center_x + 10), int(center_y)), (255, 255, 255), 1)
        cv2.line(img, (int(center_x), int(center_y - 10)), (int(center_x), int(center_y + 10)), (255, 255, 255), 1)

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
