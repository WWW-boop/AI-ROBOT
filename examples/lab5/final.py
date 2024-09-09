import cv2
from robomaster import robot, blaster, vision
import numpy as np
import time
from scipy.spatial.distance import cosine

# คลาสสำหรับเก็บข้อมูล marker(ป้าย)
class CokeMarkerInfo:
    def __init__(self, x, y, w, h, score):
        self._x = x
        self._y = y
        self._w = w
        self._h = h
        self.score = score

    @property
    def pt1(self):
        return int(self._x), int(self._y)

    @property
    def pt2(self):
        return int(self._x + self._w), int(self._y + self._h)

    @property
    def center(self):
        return int(self._x + self._w / 2), int(self._y + self._h / 2)

    @property
    def text(self):
        return f"score : {self.score:.2f}"

score_list = []

def detect_coke_can(img):
    hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_hue1 = np.array([0, 70, 50])
    upper_hue1 = np.array([10, 255, 255])
    lower_hue2 = np.array([170, 70, 50])
    upper_hue2 = np.array([180, 255, 255])
    mask1 = cv2.inRange(hsv_frame, lower_hue1, upper_hue1)
    mask2 = cv2.inRange(hsv_frame, lower_hue2, upper_hue2)
    mask = cv2.bitwise_or(mask1, mask2)

    anchor_box = [(30, 70), (50, 100), (70, 140)]
    box = None
    best_similarity = 0
    stride = 20

    for (box_w, box_h) in anchor_box:
        for y in range(0, mask.shape[0] - box_h, stride):
            for x in range(0, mask.shape[1] - box_w, stride):
                window = mask[y:y+box_h, x:x+box_w]
                window_vector = window.flatten()
                target_vector = np.ones_like(window_vector)
                window_norm = np.linalg.norm(window_vector)
                if window_norm == 0:
                    continue

                similarity = 1 - cosine(window_vector / window_norm, target_vector / np.linalg.norm(target_vector))

                if 0.6 < similarity < 0.8 and similarity > best_similarity:
                    best_similarity = similarity
                    box = (x, y, box_w, box_h)

    score_list.append(best_similarity)

    if box:
        x, y, box_w, box_h = box
        return CokeMarkerInfo(x, y, box_w, box_h, best_similarity), best_similarity

    return None, None

# ค่าเริ่มต้นสำหรับ smoothing bounding box
smooth_x, smooth_y, smooth_w, smooth_h = 0, 0, 0, 0
alpha = 0.7  # เพิ่มค่าของ alpha เพื่อให้ response เร็วขึ้น

def smooth_bbox(smooth_val, new_val, alpha):
    return smooth_val * (1 - alpha) + new_val * alpha

if __name__ == "__main__":
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_gimbal = ep_robot.gimbal
    ep_blaster = ep_robot.blaster

    center_x = 1280 / 2
    center_y = 720 / 2

    ep_camera.start_video_stream(display=False)
    ep_gimbal.sub_angle(freq=10)

    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()

    time.sleep(1)

    p = 0.5   # Higher proportional gain for faster response
    i = 0.02  # Slight increase in integral to reduce steady-state error
    d = 0.1   # Moderate derivative to dampen oscillations

    prev_err_x = 0
    prev_err_y = 0
    prev_time = time.time()

    accumulate_err_x = 0
    accumulate_err_y = 0

    data_pith_yaw = []

    detected = None

    while True:
        img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)[0:600,:]
        marker, score = detect_coke_can(img)

        if marker:
            after_time = time.time()
            x, y = marker.center

            err_x = center_x - x
            err_y = center_y - y
            accumulate_err_x += err_x * (after_time - prev_time)
            accumulate_err_y += err_y * (after_time - prev_time)

            if score > 0.75:
                if detected is None:
                    detected = (time.time()) 
                else:
                    if time.time() - detected > 2:
                        ep_blaster.fire(fire_type=blaster.INFRARED_FIRE, times=1)
                        detected = None

            speed_x = p * err_x + i * accumulate_err_x + d * (err_x - prev_err_x)
            speed_y = p * err_y + i * accumulate_err_y + d * (err_y - prev_err_y)
            ep_gimbal.drive_speed(pitch_speed=speed_y, yaw_speed=-speed_x)

            data_pith_yaw.append([err_x, err_y, round(speed_x, 3), round(speed_y, 3), after_time - prev_time])

            prev_time = after_time
            prev_err_x = err_x
            prev_err_y = err_y

            # ทำ smoothing ให้กับ bounding box
            smooth_x = smooth_bbox(smooth_x, marker.center[0], alpha)
            smooth_y = smooth_bbox(smooth_y, marker.center[1], alpha)
            smooth_w = smooth_bbox(smooth_w, marker._w, alpha)
            smooth_h = smooth_bbox(smooth_h, marker._h, alpha)

            # วาด bounding box ที่ทำ smoothing แล้ว
            cv2.rectangle(img, 
                          (int(smooth_x - smooth_w / 2), int(smooth_y - smooth_h / 2)), 
                          (int(smooth_x + smooth_w / 2), int(smooth_y + smooth_h / 2)), 
                          (0, 255, 0), 1)
    
            # แสดงข้อความของ score
            cv2.putText(img, marker.text, (int(smooth_x), int(smooth_y)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            time.sleep(0.001)

            if len(score_list) == 5:
                avg_score = sum(score_list) / len(score_list)
                print(f"Score: {avg_score:.2f}")
                score_list = []

        else:
            ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)

        cv2.imshow("Coke", img)

        # สำหรับออกจาก loop while
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()
