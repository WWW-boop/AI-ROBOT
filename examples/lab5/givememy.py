import cv2
import numpy as np
from robomaster import robot
from sklearn.metrics.pairwise import cosine_similarity

def detect_coke_can(frame, templates):
    # Noise reduction
    blurred_frame = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv_frame = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)
    
    # Improved HSV masking
    lower_hue1 = np.array([0, 50, 50])
    upper_hue1 = np.array([15, 255, 255])
    lower_hue2 = np.array([160, 50, 50])
    upper_hue2 = np.array([180, 255, 255])
    
    mask1 = cv2.inRange(hsv_frame, lower_hue1, upper_hue1)
    mask2 = cv2.inRange(hsv_frame, lower_hue2, upper_hue2)
    
    mask = mask1 | mask2
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    best_similarity = 0
    best_box = None
    
    if contours:
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = float(w) / h
            if 20 < w < 200 and 20 < h < 200 and 0.5 < aspect_ratio < 2.0:
                subregion = frame[y:y+h, x:x+w]
                for template in templates:
                    resized_template = cv2.resize(template, (w, h))
                    subregion_flat = subregion.flatten()
                    template_flat = resized_template.flatten()
                    similarity = cosine_similarity([subregion_flat], [template_flat])[0, 0]
                    
                    if similarity > best_similarity:
                        best_similarity = similarity
                        best_box = (x, y, w, h)
    
    if best_box and best_similarity > 0.5:
        x, y, w, h = best_box
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, f'Similarity: {best_similarity:.2f}', (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    return frame

def real_time_detection_with_robomaster(template_paths):
    # Load all templates
    templates = [cv2.imread(template_path) for template_path in template_paths]
    
    if any(template is None for template in templates):
        print("Error: Could not load one of the templates.")
        return
    
    # Initialize the RoboMaster robot
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    
    # Start the camera feed
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False)
    
    try:
        frame_count = 0
        while True:
            # Capture a frame from the robot's camera
            frame = ep_camera.read_cv2_image(strategy='newest', timeout=2.5)
            
            if frame is None:
                print("Error: Failed to grab frame from RoboMaster camera.")
                break
            
            # Process every 5th frame to reduce load
            if frame_count % 5 == 0:
                result_frame = detect_coke_can(frame, templates)
                cv2.imshow('Real-Time Coke Can Detection', result_frame)
            
            frame_count += 1
            
            # Exit the loop when 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    finally:
        # Stop the camera feed and close connections
        ep_camera.stop_video_stream()
        ep_robot.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    real_time_detection_with_robomaster([r'examples\pic\coke-1block.jpg',
                                         r'examples\pic\coke-3block.jpg',
                                         r'examples\pic\coke-4block.jpg'])
