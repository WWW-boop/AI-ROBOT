import cv2
import robomaster
from robomaster import robot, blaster, camera
import numpy as np
import time

# คลาสสำหรับเก็บข้อมูลตำแหน่งกระป๋องโค้ก
class CokeCanData:
    def __init__(self, x_pos, y_pos, width, height):
        self.x = x_pos
        self.y = y_pos
        self.w = width
        self.h = height

    @property
    def top_left(self):
        return int(self.x), int(self.y)

    @property
    def bottom_right(self):
        return int(self.x + self.w), int(self.y + self.h)

    @property
    def center_pos(self):
        return int(self.x + self.w // 2), int(self.y + self.h // 2)

# ฟังก์ชันสำหรับจัดการมุมของ gimbal
def handle_angle_data(angle_info):
    global angle_data
    angle_data = angle_info

# ฟังก์ชันสำหรับคำนวณความคล้ายคลึงของคอไซน์
def calc_cosine_similarity(template, window):
    dot_prod = np.sum(template * window)
    template_norm = np.linalg.norm(template)
    window_norm = np.linalg.norm(window)
    return dot_prod / (template_norm * window_norm) if template_norm > 0 and window_norm > 0 else 0

# ฟังก์ชันสำหรับเตรียม template
def load_templates():
    templates_ = [
        r"RoboMaster-SDK\examples\pic\coke-1block.jpg",
        r"RoboMaster-SDK\examples\pic\coke-100cm.jpg",
        r"RoboMaster-SDK\examples\pic\coke-3block.jpg",
        r"RoboMaster-SDK\examples\pic\coke-4block.jpg"
    ]
    template_list = [cv2.imread(temp) for temp in templates_]  # โหลดรูปภาพ
    return [crop_template(threshold_image(convert_to_hsv(temp))) for temp in template_list]

def convert_to_hsv(img):
    return cv2.cvtColor(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), cv2.COLOR_RGB2HSV)

def threshold_image(hsv_img):
    lower1, upper1 = np.array([0, 50, 50]), np.array([10, 255, 255])  # ช่วงสีแดงแรก
    lower2, upper2 = np.array([160, 50, 50]), np.array([180, 255, 255])  # ช่วงสีแดงที่สอง
    mask1, mask2 = cv2.inRange(hsv_img, lower1, upper1), cv2.inRange(hsv_img, lower2, upper2)
    return cv2.bitwise_or(mask1, mask2)

def crop_template(mask_img):
    contours, _ = cv2.findContours(mask_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
    return mask_img[y:y+h, x:x+w]

# ฟังก์ชันสำหรับ matching โดยตรง
def match_template_direct(img, templates):
    hsv_img = threshold_image(convert_to_hsv(img))  # แปลงภาพที่รับเข้ามาเป็น HSV
    best_match = (None, 0, 0, 0)  # (ตำแหน่ง, ขนาด, ความคล้าย, ค่า max_val)
    
    for i, template in enumerate(templates):
        for scale in [0.75, 1, 1.25, 1.5]:
            resized_template = cv2.resize(template, None, fx=scale, fy=scale)
            result = cv2.matchTemplate(hsv_img, resized_template, cv2.TM_CCOEFF_NORMED)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
            
            if max_val > best_match[3]:
                best_match = (max_loc, resized_template.shape[1], resized_template.shape[0], max_val)
                
    return best_match

# ฟังก์ชันหลัก
if __name__ == "__main__":
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_gimbal = ep_robot.gimbal
    ep_blaster = ep_robot.blaster

    screen_center = (1280 // 2, 720 // 2)

    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    ep_gimbal.sub_angle(freq=20, callback=handle_angle_data)

    p_gain, d_gain, i_gain = 0.353, 0.043, 0.723
    
    err_x_prev, err_y_prev = 0.0, 0.0
    time_prev = time.time()
    err_x_sum, err_y_sum = 0, 0

    templates = load_templates()
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()

    can_detected_time = None

    while True:
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        can_pos, w, h, sim_score = match_template_direct(frame, templates)

        current_time = time.time()

        if sim_score > 0.4:  # ลด threshold เพื่อเพิ่มโอกาสในการจับคู่
            x, y = can_pos
            err_x = screen_center[0] - (x + w // 2)
            err_y = screen_center[1] - (y + h // 2)

            err_x_sum += err_x
            err_y_sum += err_y

            # ควบคุม PID
            speed_x = p_gain * err_x
            speed_y = p_gain * err_y

            ep_gimbal.drive_speed(pitch_speed=speed_y, yaw_speed=-speed_x)

            if can_detected_time is None:
                can_detected_time = current_time
            elif current_time - can_detected_time > 2:
                #ep_blaster.fire(fire_type=blaster.INFRARED_FIRE, times=2)
                can_detected_time = None

            # วาดกรอบ bounding box บนภาพ
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, f"Coke Can ({sim_score:.2f})", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        else:
            ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
            can_detected_time = None

        cv2.imshow("Coke Can Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        time.sleep(0.001)

    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()
