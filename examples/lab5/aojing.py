import cv2
import numpy as np
import robomaster
from robomaster import robot, blaster, camera
import time

# คลาสสำหรับเก็บข้อมูลตำแหน่งของกระป๋อง
class CokeCanInfo:
    def __init__(self, x, y, w, h):
        self._x = x
        self._y = y
        self._w = w
        self._h = h

    @property
    def pt1(self):
        return int(self._x), int(self._y)

    @property
    def pt2(self):
        return int(self._x + self._w), int(self._y + self._h)

    @property
    def center(self):
        return int(self._x + self._w / 2), int(self._y + self._h / 2)


def cosine_similarity(template, window):
    dot_product = np.sum(template * window)
    norm_template = np.sqrt(np.sum(template**2))
    norm_window = np.sqrt(np.sum(window**2))
    if norm_template == 0 or norm_window == 0:
        return 0
    return dot_product / (norm_template * norm_window)


def prepare_templates():
    template_paths = [
        "RoboMaster-SDK\examples\pic\coke-1block.jpg",
        "RoboMaster-SDK\examples\pic\coke-100cm.jpg",
        "RoboMaster-SDK\examples\pic\coke-3block.jpg",
        "RoboMaster-SDK\examples\pic\coke-4block.jpg"
    ]

    templates = []
    for path in template_paths:
        template = cv2.imread(path)
        if template is None:
            print(f"Warning: Failed to load image at {path}")
            continue
        hsv_template = convert_to_hsv(template)
        thresholded_template = threshold_hue_channel(hsv_template)
        cropped_template, _, _ = crop_template(thresholded_template)
        templates.append(cropped_template)

    return templates


def convert_to_hsv(image):
    return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


def threshold_hue_channel(hsv_image):
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
    return cv2.bitwise_or(mask1, mask2)


def crop_template(image):
    contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return image, [], []
    largest_contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(largest_contour)
    cropped_image = image[y:y + h, x:x + w]
    return cropped_image, [(x, y)], [(w, h)]


def sliding_window_cosine_similarity(image, templates, stride=100, padding=10, scales=[0.75, 1, 1.25, 1.5]):
    max_similarity = 0
    best_position = (0, 0)
    best_size = (0, 0)

    padded_image = np.pad(image, ((padding, padding), (padding, padding), (0, 0)), mode="constant")
    hsv_image = convert_to_hsv(padded_image)
    thresholded_image = threshold_hue_channel(hsv_image)

    for scale in scales:
        for template in templates:
            scaled_template = cv2.resize(template, None, fx=scale, fy=scale)
            h, w = scaled_template.shape[:2]

            for y in range(0, thresholded_image.shape[0] - h + 1, stride):
                for x in range(0, thresholded_image.shape[1] - w + 1, stride):
                    window = thresholded_image[y:y + h, x:x + w]
                    similarity = cosine_similarity(scaled_template, window)

                    if similarity > max_similarity:
                        max_similarity = similarity
                        best_position = (x - padding, y - padding)
                        best_size = (w, h)

    return best_position, best_size, max_similarity


if __name__ == "__main__":
    # Initialize robot
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_gimbal = ep_robot.gimbal
    ep_blaster = ep_robot.blaster

    center_x = 1280 / 2
    center_y = 720 / 2

    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    ep_gimbal.sub_angle(freq=20, callback=lambda angle_info: None)

    # PID controller constants
    p = 0.353
    d = 0.043
    i = 0.723

    prev_err_x = 0.0
    prev_err_y = 0.0
    prev_time = time.time()
    accumulate_err_x = 0
    accumulate_err_y = 0

    templates = prepare_templates()
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()

    detection_start_time = None

    while True:
        img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        position, size, similarity = sliding_window_cosine_similarity(img, templates)

        current_time = time.time()

        if similarity > 0.9:  # Adjust this threshold as needed
            x, y = position
            w, h = size

            err_x = center_x - (x + w / 2)
            err_y = center_y - (y + h / 2)

            accumulate_err_x += err_x
            accumulate_err_y += err_y

            if prev_err_x and prev_err_y:
                # PID control
                speed_x = p * err_x + d * ((err_x - prev_err_x) / (current_time - prev_time)) + i * accumulate_err_x * (current_time - prev_time)
                speed_y = p * err_y + d * ((err_y - prev_err_y) / (current_time - prev_time)) + i * accumulate_err_y * (current_time - prev_time)

                ep_gimbal.drive_speed(pitch_speed=speed_y, yaw_speed=-speed_x)

            prev_err_x = err_x
            prev_err_y = err_y
            prev_time = current_time

            if detection_start_time is None:
                detection_start_time = current_time
            elif current_time - detection_start_time > 2:
                ep_blaster.fire(fire_type=blaster.INFRARED_FIRE, times=2)
                detection_start_time = None

            add_w = int(w * 0.7)
            add_h = int(h * 0.3)

            new_x = x - add_w // 2
            new_y = int(y - add_h // 1.5)
            new_w = w + add_w
            new_h = h + add_h

            cv2.rectangle(img, (new_x, new_y), (new_x + new_w, new_y + new_h), (0, 255, 0), 2)
            cv2.putText(img, f"Coke Can ({similarity:.2f})", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        else:
            ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
            detection_start_time = None

        cv2.imshow("Coke Can Detection", img)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        time.sleep(0.001)

    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()
