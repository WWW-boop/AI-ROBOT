import cv2
import numpy as np
from sklearn.metrics.pairwise import cosine_similarity

from robomaster import robot
from robomaster import vision
from robomaster import blaster
import time

# Function to compute cosine similarity between two flattened arrays
def compute_cosine_similarity(a, b):
    # Adding a small epsilon to denominator to avoid division by zero
    epsilon = 1e-10
    a_norm = np.linalg.norm(a, axis=1, keepdims=True) + epsilon
    b_norm = np.linalg.norm(b, axis=1, keepdims=True) + epsilon
    similarity = np.dot(a, b.T) / (a_norm * b_norm.T)
    return similarity

# Sliding window detection with cosine similarity
def detect_coke_can_sliding_window(frame, templates, prev_box, window_size=(100, 100), stride=100, alpha=0.2, similarity_threshold=0.5):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define color range for red (Coke can)
    lower_hue1 = np.array([0, 120, 70])  
    upper_hue1 = np.array([10, 255, 255])
    lower_hue2 = np.array([170, 150, 100])
    upper_hue2 = np.array([180, 255, 255])
    
    mask1 = cv2.inRange(hsv_frame, lower_hue1, upper_hue1)
    mask2 = cv2.inRange(hsv_frame, lower_hue2, upper_hue2)
    mask = mask1 | mask2

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    # Apply padding to the mask and frame
    pad_width = window_size[0] // 2
    padded_mask = cv2.copyMakeBorder(mask, pad_width, pad_width, pad_width, pad_width, cv2.BORDER_CONSTANT, value=0)
    padded_frame = cv2.copyMakeBorder(frame, pad_width, pad_width, pad_width, pad_width, cv2.BORDER_CONSTANT, value=0)
    
    img_height, img_width = mask.shape
    window_w, window_h = window_size
    best_similarity = 0
    best_box = None
    
    # Pre-flatten and normalize templates
    flattened_templates = []
    for template in templates:
        resized_template = cv2.resize(template, window_size)
        template_flat = resized_template.flatten().astype(np.float32)
        flattened_templates.append(template_flat)
    templates_matrix = np.array(flattened_templates)
    
    # Iterate over the image with sliding window
    for y in range(0, img_height - window_h + 1, stride):
        for x in range(0, img_width - window_w + 1, stride):
            # Check if the window has significant red color based on mask
            window_mask = padded_mask[y:y + window_h, x:x + window_w]
            red_pixels = cv2.countNonZero(window_mask)
            if red_pixels < (window_w * window_h * 0.1):  # Skip windows with less than 10% red pixels
                continue
            
            window = padded_frame[y:y + window_h, x:x + window_w]
            window_flat = window.flatten().astype(np.float32).reshape(1, -1)
            
            # Compute cosine similarity with all templates
            similarities = compute_cosine_similarity(window_flat, templates_matrix)
            max_similarity = np.max(similarities)
            max_index = np.argmax(similarities)
            
            if max_similarity > best_similarity and max_similarity > similarity_threshold:
                best_similarity = max_similarity
                best_box = (x, y, window_w, window_h)
    
    # Smoothing bounding box with previous frame
    if best_box:
        x, y, w, h = best_box
        
        # ขยายกรอบ Bounding Box ให้ครอบคลุมมากขึ้น (เพิ่ม padding)
        padding = 20  # ขนาด padding ที่จะเพิ่ม
        x = max(0, x - padding)
        y = max(0, y - padding)
        w = min(w + 2 * padding, img_width - x)
        h = min(h + 2 * padding, img_height - y)
        
        if prev_box is not None:
            prev_x, prev_y, prev_w, prev_h = prev_box
            x = int(alpha * x + (1 - alpha) * prev_x)
            y = int(alpha * y + (1 - alpha) * prev_y)
            w = int(alpha * w + (1 - alpha) * prev_w)
            h = int(alpha * h + (1 - alpha) * prev_h)
        
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, f'Similarity: {best_similarity:.2f}', (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        return frame, (x, y, w, h)
    else:
        return frame, prev_box

def process_image_sliding_window(frame, templates, prev_box):
    result_frame, updated_box = detect_coke_can_sliding_window(frame, templates, prev_box)
    return result_frame, updated_box

def sub_data_handler(angle_info):
    global list_of_data
    list_of_data = angle_info

# Main function
if __name__ == "__main__":
    
    # Initialize global variable for storing data
    list_of_data = []

    # Load templates
    templates = [
        cv2.imread(r'RoboMaster-SDK\examples\pic\coke-1block.jpg'),
        cv2.imread(r'RoboMaster-SDK\examples\pic\coke-3block.jpg'),
        cv2.imread(r'RoboMaster-SDK\examples\pic\coke-4block.jpg')
    ]

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    
    ep_camera = ep_robot.camera
    ep_gimbal = ep_robot.gimbal
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()
    ep_camera.start_video_stream(display=False)
    

    # Image center constants
    center_x = 1280 / 2
    center_y = 720 / 2

    # PID controller constants
    p = -0.607  # -0.609 -0.65
    i = 0
    d = -0.00135

    accumulate_err_x = 0
    accumulate_err_y = 0
    data_pith_yaw = []
    prev_box = None  # Store bounding box from previous frame
    alpha = 0.2  # Smoothing factor
    count = 0
    prev_time = time.time()
    prev_err_x = 0
    prev_err_y = 0

    while True:
        current_time = time.time()
        img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        
        # Detect Coke can using sliding window and apply smoothing to bounding box
        result_frame, prev_box = process_image_sliding_window(img, templates, prev_box)

        if prev_box is not None:
            x, y, w, h = prev_box
            err_x = center_x - (x + w / 2)
            err_y = center_y - (y + h / 2)
            accumulate_err_x += err_x
            accumulate_err_y += err_y
            
            if count >= 1:
                # Compute time difference
                time_diff = current_time - prev_time if current_time - prev_time > 0 else 1e-5

                # Calculate speed using PID controller
                speed_x = (
                    (p * err_x)
                    + (d * (err_x - prev_err_x) / time_diff)
                    + (i * accumulate_err_x)
                )
                speed_y = (
                    (p * err_y)
                    + (d * (err_y - prev_err_y) / time_diff)
                    + (i * accumulate_err_y)
                )
                ep_gimbal.drive_speed(pitch_speed=-speed_y, yaw_speed=speed_x)
                data_pith_yaw.append(
                    list(list_of_data)  # Ensure list_of_data is updated
                    + [err_x, err_y, round(speed_x, 3), round(speed_y, 3)]
                )

            count += 1
            prev_time = current_time
            prev_err_x = err_x
            prev_err_y = err_y
            time.sleep(0.001)
        else:
            # Reset gimbal to center if no detection
            ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
        
        # Display the result
        cv2.imshow("Coke Can Detection", result_frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()
