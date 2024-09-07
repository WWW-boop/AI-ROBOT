import cv2
import robomaster
from robomaster import robot, blaster, camera
import time
import numpy as np
import matplotlib.pyplot as plt


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


# เก็บข้อมูลมุมของ gimbal
def sub_data_handler(angle_info):
    global list_of_data
    list_of_data = angle_info


def cosine_similarity(template, window):
    dot_product = np.sum(template * window)
    norm_template = np.sqrt(np.sum(template**2))
    norm_window = np.sqrt(np.sum(window**2))
    if norm_template == 0 or norm_window == 0:
        return 0
    return dot_product / (norm_template * norm_window)


def prepare_templates():
    template_paths = [
        r'RoboMaster-SDK\examples\pic\1.png',
        r'RoboMaster-SDK\examples\pic\2.png',
        r'RoboMaster-SDK\examples\pic\3.png',
        r'RoboMaster-SDK\examples\pic\4.png',
    ]

    templates = []
    for path in template_paths:
        template = cv2.imread(path)
        # if template is None:
        #     print(f"Warning: Could not read template image at {path}")
        #     continue
        hsv_template = convert_to_hsv(template)
        thresholded_template = threshold_hue_channel(hsv_template)
        cropped_template, _, _ = crop_template(thresholded_template)
        templates.append(cropped_template)

    return templates


def convert_to_hsv(image):
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)
    return hsv_image


def threshold_hue_channel(hsv_image):
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    return mask


def crop_template(image):
    contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest_contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(largest_contour)
    cropped_image = image[y : y + h, x : x + w]
    return cropped_image, [(x, y)], [(w, h)]


def sliding_window_cosine_similarity(
    image, templates, stride=100, padding=10, scales=[1, 2]
):
    max_similarity = 0
    best_position = (0, 0)
    best_template_size = (0, 0)

    padded_image = np.pad(
        image, ((padding, padding), (padding, padding), (0, 0)), mode="constant"
    )
    t_convert_to_hsv = convert_to_hsv(padded_image)
    t_threshold_hue_channel = threshold_hue_channel(t_convert_to_hsv)

    for scale in scales:
        for template in templates:
            scaled_template = cv2.resize(template, None, fx=scale, fy=scale)
            h, w = scaled_template.shape[:2]
            print(f"Using template size: {w}x{h} for scale {scale}")
            for y in range(0, t_threshold_hue_channel.shape[0] - h + 1, stride):
                for x in range(0, t_threshold_hue_channel.shape[1] - w + 1, stride):
                    window = t_threshold_hue_channel[y : y + h, x : x + w]
                    similarity = cosine_similarity(scaled_template, window)
                    if similarity > max_similarity:
                        max_similarity = similarity
                        best_position = (x - padding, y - padding)
                        best_template_size = (w, h)

    final_position = (best_position[0] - padding, best_position[1] - padding)
    return final_position, best_template_size, max_similarity


if __name__ == "__main__":
    # initialize robot
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_gimbal = ep_robot.gimbal
    ep_blaster = ep_robot.blaster

    # the image center constants
    center_x = 1280 / 2
    center_y = 720 / 2

    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    ep_gimbal.sub_angle(freq=20, callback=sub_data_handler)

    # PID controller constants
    # p = 0.3529  # 0.4705
    # d = 0.0316
    # i = 0.9856

    p = 0
    d = 0
    i = 0

    prev_err_x = 0.0
    prev_err_y = 0.0
    prev_time = time.time()

    accumulate_err_x = 0
    accumulate_err_y = 0

    data_pitch_yaw = []

    count = 0

    templates = prepare_templates()
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()

    detection_start_time = None

    while True:
        img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        position, size, similarity = sliding_window_cosine_similarity(img, templates)

        current_time = time.time()  # บันทึกเวลาปัจจุบันในลูป

        if similarity:  # Coke can found
            after_time = time.time()
            x, y = position
            w, h = size

            err_x = center_x - x
            err_y = center_y - y

            accumulate_err_x += err_x
            accumulate_err_y += err_y

            if count >= 1:
                # PID control
                speed_x = (
                    (p * err_x)
                    + d * ((err_x - prev_err_x) / (after_time - prev_time))
                    + i * (accumulate_err_x) * (after_time - prev_time)
                )
                speed_y = (
                    (p * err_y)
                    + d * ((err_y - prev_err_y) / (after_time - prev_time))
                    + i * (accumulate_err_y) * (after_time - prev_time)
                )

                ep_gimbal.drive_speed(pitch_speed=speed_y, yaw_speed=-speed_x)
                # if abs(err_x) <= 5 and abs(err_y) <= 5:
                # ep_blaster.fire(fire_type=blaster.INFRARED_FIRE, times=2)

                data_pitch_yaw.append(
                    [current_time]  # บันทึกเวลาใน list
                    + list(list_of_data)
                    + [err_x, err_y, round(speed_x, 3), round(speed_y, 3)]
                    + [x, y]
                )

            count += 1
            prev_err_x = err_x
            prev_err_y = err_y
            prev_time = after_time

            time.sleep(0.001)

        else:
            ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
            detection_start_time = None  # Reset detection start time when not detected

        if similarity > 0.9:
            if detection_start_time is None:
                detection_start_time = (
                    time.time()
                )  # Start timing when the bounding box first appears
            else:
                if time.time() - detection_start_time > 2:
                    ep_blaster.fire(fire_type=blaster.INFRARED_FIRE, times=2)
                    detection_start_time = None  # Reset after firing

            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0))
            cv2.putText(
                img,
                "Coke Can",
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.5,
                (0, 255, 0),
                3,
            )
        # else:
        #     ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)

        # if similarity > 0.9:
        #     start_gimbal = time.time()
        #     cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0))
        #     cv2.putText(
        #         img,
        #         "Coke Can",
        #         (x, y - 10),
        #         cv2.FONT_HERSHEY_SIMPLEX,
        #         1.5,
        #         (0, 255, 0),
        #         3,
        #     )
        #     end_gimbal = time.time()
        #     if abs(start_gimbal - end_gimbal) > 2:
        #         ep_blaster.fire(fire_type=blaster.INFRARED_FIRE, times=2)

        cv2.imshow("Coke Can Detection", img)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()

    # plot error x, error y, speed x, speed y
    # x_point = [i for i in range(len(data_pitch_yaw))]
    x_point = [
        i[0] - data_pitch_yaw[0][0] for i in data_pitch_yaw
    ]  # คำนวณค่า time step โดยอิงจากเวลาที่บันทึก
    y_point4 = [i[4] for i in data_pitch_yaw]
    y_point5 = [i[5] for i in data_pitch_yaw]
    y_point6 = [i[6] for i in data_pitch_yaw]
    y_point7 = [i[7] for i in data_pitch_yaw]
    y_point8 = [i[8] for i in data_pitch_yaw]  # x
    y_point9 = [i[9] for i in data_pitch_yaw]  # y
    print("err_x =", y_point4)
    # x_point = [i for i in range(list(list_of_data))]
    # print(x_point)
    # y_point = [i[1] for i in list(list_of_data)]
    plt.plot(x_point, y_point4)

    plt.legend(["gimbal"])
    plt.show()