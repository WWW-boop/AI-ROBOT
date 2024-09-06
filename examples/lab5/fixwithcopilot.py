import cv2
import numpy as np
from sklearn.metrics.pairwise import cosine_similarity
from robomaster import robot, vision, blaster
import time

# Function to compute cosine similarity between two flattened arrays
def compute_cosine_similarity(a, b):
    epsilon = 1e-10
    a_norm = np.linalg.norm(a, axis=1, keepdims=True) + epsilon
    b_norm = np.linalg.norm(b, axis=1, keepdims=True) + epsilon
    similarity = np.dot(a, b.T) / (a_norm * b_norm.T)
    return similarity

# Sliding window detection with cosine similarity
def detect_coke_can_sliding_window(frame, templates, prev_box, window_size=(150, 150), stride=50, alpha=0.2, similarity_threshold=0.5):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
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
    
    pad_width = window_size[0] // 2
    padded_mask = cv2.copyMakeBorder(mask, pad_width, pad_width, pad_width, pad_width, cv2.BORDER_CONSTANT, value=0)
    padded_frame = cv2.copyMakeBorder(frame, pad_width, pad_width, pad_width, pad_width, cv2.BORDER_CONSTANT, value=0)
    
    img_height, img_width = mask.shape
    window_w, window_h = window_size
    best_similarity = 0
    best_box = None
    
    flattened_templates = []
    for template in templates:
        resized_template = cv2.resize(template, window_size)
        template_flat = resized_template.flatten().astype(np.float32)
        flattened_templates.append(template_flat)
    templates_matrix = np.array(flattened_templates)
    
    for y in range(0, img_height - window_h + 1, stride):
        for x in range(0, img_width - window_w + 1, stride):
            window_mask = padded_mask[y:y + window_h, x:x + window_w]
            red_pixels = cv2.countNonZero(window_mask)
            if red_pixels < (window_w * window_h * 0.1):
                continue
            
            window = padded_frame[y:y + window_h, x:x + window_w]
            window_flat = window.flatten().astype(np.float32).reshape(1, -1)
            
            similarities = compute_cosine_similarity(window_flat, templates_matrix)
            max_similarity = np.max(similarities)
            
            if max_similarity > best_similarity and max_similarity > similarity_threshold:
                best_similarity = max_similarity
                best_box = (x, y, window_w, window_h)
    
    if best_box:
        x, y, w, h = best_box
        if prev_box is not None:
            prev_x, prev_y, prev_w, prev_h = prev_box
            x = int(alpha * x + (1 - alpha) * prev_x)
            y = int(alpha * y + (1 - alpha) * prev_y)
            w = int(alpha * w + (1 - alpha) * prev_w)
            h = int(alpha * h + (1 - alpha) * prev_h)
        
        # Adjust the bounding box to better fit the Coke can
        x = max(0, x - 10)
        y = max(0, y - 10)
        w = min(frame.shape[1] - x, w + 20)
        h = min(frame.shape[0] - y, h + 20)
        
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
    
    list_of_data = []

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
    
    center_x = 1280 / 2
    center_y = 720 / 2

    p = -0.607
    i = 0
    d = -0.00135

    accumulate_err_x = 0
    accumulate_err_y = 0
    data_pith_yaw = []
    prev_box = None
    alpha = 0.2
    count = 0
    prev_time = time.time()
    prev_err_x = 0
    prev_err_y = 0

    while True:
        current_time = time.time()
        img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        
        result_frame, prev_box = process_image_sliding_window(img, templates, prev_box)

        if prev_box is not None:
            x, y, w, h = prev_box
            err_x = center_x - (x + w / 2)
            err_y = center_y - (y + h / 2)
            accumulate_err_x += err_x
            accumulate_err_y += err_y
            
            if count >= 1:
                time_diff = current_time - prev_time if current_time - prev_time > 0 else 1e-5

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
                    list(list_of_data)
                    + [err_x, err_y, round(speed_x, 3), round(speed_y, 3)]
                )

            count += 1
            prev_time = current_time
            prev_err_x = err_x
            prev_err_y = err_y
            time.sleep(0.001)
        else:
            ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
        
        cv2.imshow("Coke Can Detection", result_frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()
