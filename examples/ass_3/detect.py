import cv2
import robomaster
from robomaster import robot, camera
import time
import numpy as np

def blue_head_culprit(hsv, img):
    # กำหนดค่าช่วงสีฟ้าสำหรับฝาใน HSV
    lower_hue_bottle = np.array([90, 50, 50])
    upper_hue_bottle = np.array([130, 255, 255])

    # ตรวจจับบริเวณที่มีสีฟ้า
    bottle_mask = cv2.inRange(hsv, lower_hue_bottle, upper_hue_bottle)

    # กำจัด noise โดยใช้ Morphological Transformations
    kernel = np.ones((5, 5), np.uint8)
    bottle_mask = cv2.morphologyEx(bottle_mask, cv2.MORPH_CLOSE, kernel)
    bottle_mask = cv2.morphologyEx(bottle_mask, cv2.MORPH_OPEN, kernel)

    # เพิ่ม Gaussian Blur เพื่อช่วยลด noise
    bottle_mask = cv2.GaussianBlur(bottle_mask, (5, 5), 0)

    # หาขอบเขตของบริเวณที่เป็นฝาสีฟ้า
    bottle_contours, _ = cv2.findContours(bottle_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(bottle_contours) > 0:
        bottle_contour_max = max(bottle_contours, key=cv2.contourArea)
        bottle_x, bottle_y, bottle_width, bottle_height = cv2.boundingRect(bottle_contour_max)

        # ตรวจสอบขนาดของ bounding box
        if bottle_width > 20 and bottle_height > 20:  # ขนาดขั้นต่ำของ bounding box
            # ปรับขนาดของ bounding box โดยอัตโนมัติ
            width_factor = 0.3
            height_factor = 2.8

            adjust_width = int(bottle_width * width_factor)
            adjust_height = int(bottle_height * height_factor)

            new_bottle_x = bottle_x - adjust_width // 2
            new_bottle_y = bottle_y - adjust_height // 2
            new_bottle_width = bottle_width + adjust_width
            new_bottle_height = bottle_height + adjust_height

            # วาด bounding box รอบฝาและแสดงข้อความ
            cv2.rectangle(img, 
                          (new_bottle_x, new_bottle_y), 
                          (new_bottle_x + new_bottle_width, new_bottle_y + new_bottle_height), 
                          (0, 255, 0), 2)

            cv2.putText(img, "Bottle Detected", 
                        (new_bottle_x, new_bottle_y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
    else:
        # ถ้าไม่เจอให้แสดงข้อความว่าไม่เจอฝา
        cv2.putText(img, "No Bottle Detected", 
                    (20, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    return img

def detect_acrylic_culprit(img):
    # Convert the image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Apply GaussianBlur to reduce noise and improve edge detection
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Use Canny Edge Detection
    edges = cv2.Canny(blurred, 50, 150)
    
    # Find contours from the edges detected
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Find the largest contour based on the area
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Draw bounding box around the detected acrylic figure
        x, y, w, h = cv2.boundingRect(largest_contour)
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Display a message indicating detection
        cv2.putText(img, "Acrylic Detected", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    return img

def feature_descriptor_detection(img):
    # Convert to grayscale for feature detection
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Use ORB (Oriented FAST and Rotated BRIEF) detector
    orb = cv2.ORB_create()

    # Detect keypoints and descriptors
    keypoints, descriptors = orb.detectAndCompute(gray, None)

    # Draw keypoints on the image
    img_with_keypoints = cv2.drawKeypoints(img, keypoints, None, color=(0, 255, 0), flags=0)

    return img_with_keypoints

def shape_culprit(img):
    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Apply GaussianBlur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Detect edges using Canny
    edges = cv2.Canny(blurred, 50, 150)

    # Find contours
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Loop through contours and find those matching certain shapes
        for contour in contours:
            # Approximate the contour to simplify the shape
            approx = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True)

            # Check if the contour has enough points to form a shape (e.g., person-like shape)
            if len(approx) >= 5:  # Assumes the figure is complex enough to have multiple edges
                # Draw the contour (representing the shape)
                cv2.drawContours(img, [contour], -1, (0, 255, 0), 2)
                cv2.putText(img, "Shape Detected", (approx[0][0][0], approx[0][0][1] - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    return img

def main():
    # Initialize the RoboMaster robot
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    
    # Access the camera
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False)

    try:
        while True:
            # Capture image from RoboMaster camera
            frame = ep_camera.read_cv2_image(strategy="newest")
            
            # Convert the image to HSV for color detection
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Detect blue bottle cap, acrylic figure, features, and shapes
            frame_with_bottle = blue_head_culprit(hsv, frame)
            frame_with_acrylic = detect_acrylic_culprit(frame_with_bottle)
            frame_with_features = feature_descriptor_detection(frame_with_acrylic)
            final_frame = shape_culprit(frame_with_features)
            
            # Display the resulting frame with detected objects
            cv2.imshow("Detection", final_frame)
            
            # Press 'q' to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        ep_camera.stop_video_stream()
        ep_robot.close()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
