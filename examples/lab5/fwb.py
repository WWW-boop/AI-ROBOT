import cv2
import numpy as np
from sklearn.metrics.pairwise import cosine_similarity
from robomaster import robot, vision, blaster
import time

# ฟังก์ชันตรวจจับ Coke can ด้วยการ smoothin
def detect_coke_can(frame, templates, prev_box, alpha=0.2):
    best_similarity = 0
    best_box = None

    # ใช้ sliding window แทนการหา contours
    window_size = (100, 100)  # ขนาดของ sliding window
    step_size = 10  # การเลื่อนหน้าต่าง

    for y in range(0, frame.shape[0] - window_size[1], step_size):
        for x in range(0, frame.shape[1] - window_size[0], step_size):
            subregion = frame[y:y+window_size[1], x:x+window_size[0]]
            
            # เปรียบเทียบ subregion กับ templates
            for template in templates:
                resized_template = cv2.resize(template, window_size)
                subregion_flat = subregion.flatten()
                template_flat = resized_template.flatten()
                similarity = cosine_similarity([subregion_flat], [template_flat])[0, 0]
                
                if similarity > best_similarity:
                    best_similarity = similarity
                    best_box = (x, y, window_size[0], window_size[1])
    
    # Smoothing bounding box ด้วย weighted average
    if best_box and best_similarity > 0.5:
        x, y, w, h = best_box
        
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


def process_image(frame, templates, prev_box):
    result_frame, updated_box = detect_coke_can(frame, templates, prev_box)
    return result_frame, updated_box

def sub_data_handler(angle_info):
    global list_of_data
    list_of_data = angle_info

# ฟังก์ชันหลัก
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
    ep_camera.start_video_stream(display=False)

    # the image center constants
    center_x = 1280 / 2
    center_y = 720 / 2

    # PID controller constants
    p = -0.607  # -0.609 -0.65
    i = 0
    d = -0.00135

    accumulate_err_x = 0
    accumulate_err_y = 0
    data_pith_yaw = []
    prev_box = None  # เก็บ bounding box จากเฟรมก่อนหน้า
    alpha = 0.2  # ปัจจัยการ smooth
    count = 0

    while True:
        after_time = time.time()
        img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        
        # ตรวจจับ Coke และทำ smoothing bounding box
        result_frame, prev_box = process_image(img, templates, prev_box)

        if prev_box is not None:
            x, y, w, h = prev_box
            err_x = center_x - x
            err_y = center_y - y
            accumulate_err_x += err_x
            accumulate_err_y += err_y
            
            if count >= 1:
                # คำนวณความเร็วในการหมุน gimbal โดยใช้ PID
                speed_x = (
                    (p * err_x)
                    + d * ((prev_err_x - err_x) / (prev_time - after_time))
                    + i * (accumulate_err_x)
                )
                speed_y = (
                    (p * err_y)
                    + d * ((prev_err_y - err_y) / (prev_time - after_time))
                    + i * (accumulate_err_y)
                )
                ep_gimbal.drive_speed(pitch_speed=-speed_y, yaw_speed=speed_x)
                data_pith_yaw.append(
                    list(list_of_data)  # Ensure list_of_data is updated
                    + [err_x, err_y, round(speed_x, 3), round(speed_y, 3)]
                )

            count += 1
            prev_time = time.time()
            prev_err_x = err_x
            prev_err_y = err_y
            time.sleep(0.001)
        else:
            # หมุนกลับ center
            ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
        
        # แสดงผล
        cv2.imshow("Coke Can Detection", result_frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()
