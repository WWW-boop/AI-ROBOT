import cv2
import robomaster
from robomaster import robot
from robomaster import vision
from robomaster import blaster
import time
import numpy as np
from scipy.spatial.distance import cosine
import matplotlib.pyplot as plt

## 7.2 -7.8
lst_score = []

class CokeCanMarker:
    def __init__(self, x, y, w, h, score):
        self._x = x
        self._y = y
        self._w = w
        self._h = h
        self._score = score

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
        return f"Score: {self._score:.2f}"

def detect_coke_can(image):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 70, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 70, 50])
    upper_red2 = np.array([180, 255, 255])
    
    mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    
    anchor_boxes = [(30, 70)]
    best_score = -1
    best_box = None
    stride = 10
    
    for (box_w, box_h) in anchor_boxes:
        for y in range(0, mask.shape[0] - box_h, stride):
            for x in range(0, mask.shape[1] - box_w, stride):
                window = mask[y:y+box_h, x:x+box_w]
                window_vector = window.flatten()
                target_vector = np.ones_like(window_vector)
                
                window_norm = np.linalg.norm(window_vector)
                target_norm = np.linalg.norm(target_vector)
                if window_norm == 0 or target_norm == 0:
                    continue
                
                similarity = 1 - cosine(window_vector / window_norm, target_vector / target_norm)
                
                if 0.72 < similarity < 0.78 and similarity > best_score:
                    best_score = similarity
                    best_box = (x, y, box_w, box_h)
                    
    lst_score.append(best_score)

    if best_box:
        x, y, box_w, box_h = best_box
        cv2.rectangle(image, (x, y), (x + box_w, y + box_h), (0, 255, 0), 2)
        return CokeCanMarker(x, y, box_w, box_h, best_score), best_score

    return None, None


if __name__ == "__main__":
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_blaster = ep_robot.blaster
    ep_camera = ep_robot.camera
    ep_gimbal = ep_robot.gimbal

    center_x = 1280 / 2
    center_y = 720 / 2
    ep_camera.start_video_stream(display=False)
    ep_gimbal.sub_angle(freq=10)
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()
    time.sleep(1)

    p = 0.6 / 1.7
    i = p / (0.7 / 2)
    d = p * (0.7 / 8)
    boom = True

    accumulate_err_x = 0
    accumulate_err_y = 0
    data_pith_yaw = []
    prev_time = time.time()
    prev_err_x = 0
    prev_err_y = 0
    
    while True:
        img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        marker,score = detect_coke_can(img)

        if marker:
            after_time = time.time()
            x, y = marker.center

            err_x = center_x - x
            err_y = center_y - y
            accumulate_err_x += err_x * (after_time - prev_time)
            accumulate_err_y += err_y * (after_time - prev_time)

            if boom and 0.5 <= err_x and err_y <= 1:
                ep_blaster.fire(times=1)
                boom = False

            speed_x = p * err_x
            speed_y = p * err_y
            ep_gimbal.drive_speed(pitch_speed=speed_y, yaw_speed=-speed_x)

            data_pith_yaw.append([err_x, err_y, round(speed_x, 3), round(speed_y, 3), after_time - prev_time])

            prev_time = after_time
            prev_err_x = err_x
            prev_err_y = err_y

            cv2.rectangle(img, marker.pt1, marker.pt2, (0, 255, 0), 2)
            cv2.putText(img, marker.text, marker.center, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            time.sleep(0.001)

            if len(lst_score) == 10 :
                result = sum(lst_score) / len(lst_score)
                print(result)
                lst_score = []

        else:
            ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)

        cv2.imshow("Coke Can Detection", img)
        


        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()

    x_point = [i for i in range(len(data_pith_yaw))]
    y_point4 = [i[4] for i in data_pith_yaw]


    plt.plot(x_point, y_point4)
    plt.legend(["e x", "e y", "u x", "u y"])
    plt.show()
    plt.savefig("graph.png")
