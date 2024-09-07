import cv2
import robomaster
from robomaster import robot
from robomaster import vision
from robomaster import blaster
import numpy as np
import time
import pandas as pd
import matplotlib.pyplot as plt


# คลาสสำหรับเก็บข้อมูล marker(ป้าย)
class MarkerInfo:
    # ข้อมูลจุดตรงกลางป้าย ความกว้าง ความยาว ข้อมูลป้าย
    def __init__(self, x, y, w, h, info):
        self._x = x
        self._y = y
        self._w = w
        self._h = h

    # คำนวณมุมซ้ายบนของป้ายและแปลงเป็น pixel
    @property
    def pt1(self):
        return int((self._x - self._w / 2) * 1280), int((self._y - self._h / 2) * 720)

    # คำนวณมุมขวาล่างของป้ายและแปลงเป็น pixel
    @property
    def pt2(self):
        return int((self._x + self._w / 2) * 1280), int((self._y + self._h / 2) * 720)

    # จุดกลางป้ายเป็น pixel
    @property
    def center(self):
        return int(self._x * 1280), int(self._y * 720)

templates = []

def convert_rgb_to_hsv(frame):
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    hsv_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2HSV)
    return hsv_frame

def threshold(frame):
    lower_hue1 = np.array([0, 120, 70])  
    upper_hue1 = np.array([10, 255, 255])
    lower_hue2 = np.array([170, 150, 100])
    upper_hue2 = np.array([180, 255, 255])
    
    mask1 = cv2.inRange(frame, lower_hue1, upper_hue1)
    mask2 = cv2.inRange(frame, lower_hue2, upper_hue2)
    mask = mask1 | mask2

    return mask

def anchor_box(image):
    image = threshold(image)
    contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # ดึงเฉพาะ contours
    if len(contours) == 0:
        return None, None, None  # ถ้าไม่มี contours ให้คืนค่า None เพื่อจัดการข้อผิดพลาด
    contour = max(contours, key=cv2.contourArea)  # เลือก contour ที่มีพื้นที่ใหญ่ที่สุด
    x, y, w, h = cv2.boundingRect(contour)
    cropped_image = image[y : y + h, x : x + w]
    return cropped_image, [(x, y)], [(w, h)]


def template(image_paths):
    templates = []
    for image_path in image_paths:
        image = cv2.imread(image_path)
        if image is None:
            print(f"Error: Failed to load image {image_path}")
            continue  # Skip to the next image if loading failed
        result = anchor_box(image)
        if result and result[0] is not None:
            cropped_image = result[0]
            templates.append(cropped_image)
    return templates



def cosine_similarity(template, window):
    dot_product = np.sum(template * window)
    norm_template = np.sqrt(np.sum(template**2))
    norm_window = np.sqrt(np.sum(window**2))
    if norm_template == 0 or norm_window == 0:
        return 0
    return dot_product / (norm_template * norm_window)

def compute_consine_smilarity_with_sliding_window(frame, templates, stride=100, padding=10, scales=[0.5, 0.75, 1, 1.25, 1.5, 1.75]):
    max_similarity = 0
    best_position = (0, 0)
    best_size = (0, 0)

    padded_image = np.pad(frame, ((padding, padding), (padding, padding), (0, 0)), mode="constant")
    t_convert_to_hsv = convert_rgb_to_hsv(padded_image)
    t_threshold_hue_channel = threshold(t_convert_to_hsv)

    for scale in scales:
        for template_image in templates:
            if template_image is None or template_image.size == 0:
                print("Error: Invalid template image, skipping.")
                continue  # Skip invalid templates
            
            scaled_template = cv2.resize(template_image, None, fx=scale, fy=scale)
            h, w = scaled_template.shape[:2]

            for y in range(0, t_threshold_hue_channel.shape[0] - h + 1, stride):
                for x in range(0, t_threshold_hue_channel.shape[1] - w + 1, stride):
                    window = t_threshold_hue_channel[y : y + h, x : x + w]
                    similarity = cosine_similarity(scaled_template, window)

                    if similarity > max_similarity:
                        max_similarity = similarity
                        best_position = (x - padding - 10, y - padding - 10)
                        best_size = (w, h)

    return best_position, best_size, max_similarity


# เก็บข้อมูลมุมของ gimbal
def sub_data_handler(angle_info):
    global list_of_data
    list_of_data = angle_info


if __name__ == "__main__":
    # initialize robot
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_vision = ep_robot.vision
    ep_camera = ep_robot.camera
    ep_gimbal = ep_robot.gimbal
    ep_blaster = ep_robot.blaster

    # the image center constants
    center_x = 1280 / 2
    center_y = 720 / 2

    count = 0

    ep_camera.start_video_stream(display=False)
    ep_gimbal.sub_angle(freq=50, callback=sub_data_handler)

    # หมุน gimbal กลับไปที่ center
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()

    templates_path = [
        r'241-251\AI-ROBOT\examples\pic\coke-1block.jpg',
        r'241-251\AI-ROBOT\examples\pic\coke-100cm.jpg',
        r'241-251\AI-ROBOT\examples\pic\coke-3block.jpg',
        r'241-251\AI-ROBOT\examples\pic\coke-4block.jpg'
    ]
    templates = template(templates_path)

    # PID controller constants
    p = -0.607  # -0.609 -0.65
    i = 0
    d = -0.00135

    data_pith_yaw = []

    # loop การทำงานของหุ่น
    while True:

        img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        position, size, similarity = compute_consine_smilarity_with_sliding_window(img, templates)

        current_time = time.time()

        if similarity > 0.8:  # target found
            after_time = time.time()
            x, y =  position
            w, h = size

            err_x = (center_x - x) 
            err_y = (center_y - y) 

            if count >= 1:
                # คำนวณความเร็วในการหมุน gimbal โดยใช้ PID
                speed_x = (
                    (p * err_x)
                )
                speed_y = (
                    (p * err_y)
                )

                # หมุน gimbal ตามความเร็วที่คำนวณมาก
                ep_gimbal.drive_speed(pitch_speed=-speed_y, yaw_speed=speed_x)

                # เก็บค่ามุมของ gimbal, error x, error y, speed x, speed y
                data_pith_yaw.append(
                    list(list_of_data)
                    + [err_x, err_y, round(speed_x, 3), round(speed_y, 3)]
                )
            count+=1
            add_w = int(w * 0.7)
            add_h = int(h * 0.3)

            new_x = x - add_w // 2
            new_y = int(y - add_h // 1.5)
            new_w = w + add_w
            new_h = h + add_h

            cv2.rectangle(
                img, (new_x, new_y), (new_x + new_w, new_y + new_h), (0, 255, 0), 2
            )
            cv2.putText(
                img,
                f"Coke Can ({similarity:.2f})",
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 255, 0),
                2,
            )

        else:
            # หมุนกลับ center
            ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)

        # สำหรับออกจาก loop while
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()

    result = ep_vision.unsub_detect_info(name="marker")
    ep_camera.stop_video_stream()
    ep_robot.close()
