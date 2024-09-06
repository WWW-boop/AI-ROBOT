import cv2
import numpy as np
from sklearn.metrics.pairwise import cosine_similarity
from robomaster import robot
import time

# ฟังก์ชันตรวจจับ small chick plush toy ด้วยการ smoothing
def detect_chick(frame, templates, prev_box, alpha=0.2):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define yellow color range in HSV
    lower_yellow = np.array([20, 100, 100])  # Modify these values as needed to detect the plush toy
    upper_yellow = np.array([30, 255, 255])  # Modify these values as needed to detect the plush toy
    
    mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    best_similarity = 0
    best_box = None
    
    if contours:
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w > 20 and h > 20:
                subregion = frame[y:y+h, x:x+w]
                
                for template in templates:
                    resized_template = cv2.resize(template, (w, h))
                    subregion_flat = subregion.flatten()
                    template_flat = resized_template.flatten()
                    similarity = cosine_similarity([subregion_flat], [template_flat])[0, 0]
                    
                    if similarity > best_similarity:
                        best_similarity = similarity
                        best_box = (x, y, w, h)
    
    # Smoothing bounding box ด้วย weighted average
    if best_box and best_similarity > 0.5:
        x, y, w, h = best_box
        
        if prev_box is not None:
            prev_x, prev_y, prev_w, prev_h = prev_box
            
            # Smooth ตำแหน่งของ bounding box
            x = int(alpha * x + (1 - alpha) * prev_x)
            y = int(alpha * y + (1 - alpha) * prev_y)
            
            # Smooth ขนาดของ bounding box ด้วย alpha ขนาดเล็กลง
            size_alpha = 0.1  # ใช้ค่า alpha ที่เล็กลงสำหรับการเปลี่ยนขนาด
            w = int(size_alpha * w + (1 - size_alpha) * prev_w)
            h = int(size_alpha * h + (1 - size_alpha) * prev_h)
        
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, f'Similarity: {best_similarity:.2f}', (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        return frame, (x, y, w, h)
    else:
        return frame, prev_box

def process_image(frame, templates, prev_box):
    result_frame, updated_box = detect_chick(frame, templates, prev_box)
    return result_frame, updated_box

# ฟังก์ชันหลัก
if __name__ == "__main__":
    # Load templates (replace these with images of the chick plush toy)
    templates = [
        cv2.imread(r'RoboMaster-SDK\examples\pic\chick-1block.jpg'),
        cv2.imread(r'RoboMaster-SDK\examples\pic\chick-3block.jpg'),
        cv2.imread(r'RoboMaster-SDK\examples\pic\chick-4block.jpg')
    ]

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_gimbal = ep_robot.gimbal
    ep_gimbal.recenter().wait_for_completed()
    ep_camera.start_video_stream(display=False)

    # the image center constants
    center_x = 1280 / 2
    center_y = 720 / 2

    # PID controller constants
    p = -0.607
    i = 0
    d = -0.00135

    accumulate_err_x = 0
    accumulate_err_y = 0
    prev_box = None
    alpha = 0.2
    prev_time = time.time()
    prev_err_x = 0
    prev_err_y = 0

    while True:
        after_time = time.time()
        img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        
        if img is not None:
            # ตรวจจับ small chick plush toy และทำ smoothing bounding box
            result_frame, prev_box = process_image(img, templates, prev_box)

            if prev_box is not None:
                x, y, w, h = prev_box
                err_x = center_x - (x + w / 2)
                err_y = center_y - (y + h / 2)
                accumulate_err_x += err_x
                accumulate_err_y += err_y

                if prev_time != after_time:
                    # คำนวณความเร็วในการหมุน gimbal โดยใช้ PID
                    speed_x = (
                        (p * err_x)
                        + d * ((prev_err_x - err_x) / (after_time - prev_time))
                        + i * accumulate_err_x
                    )
                    speed_y = (
                        (p * err_y)
                        + d * ((prev_err_y - err_y) / (after_time - prev_time))
                        + i * accumulate_err_y
                    )
                    ep_gimbal.drive_speed(pitch_speed=-speed_y, yaw_speed=speed_x)

                prev_time = after_time
                prev_err_x = err_x
                prev_err_y = err_y
            else:
                # หมุนกลับ center
                ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)

            # แสดงผล
            cv2.imshow("Chick Detection", result_frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        else:
            print("Failed to get image from camera")

    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()
