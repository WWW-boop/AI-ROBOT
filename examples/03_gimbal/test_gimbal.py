import cv2
import robomaster
from robomaster import robot
from robomaster import vision
from robomaster import blaster
import time
import pandas as pd
import matplotlib.pyplot as plt


class MarkerInfo:

    def __init__(self, x, y, w, h, info):
        self._x = x
        self._y = y
        self._w = w
        self._h = h
        self._info = info

    @property
    def pt1(self):
        return int((self._x - self._w / 2) * 1280), int((self._y - self._h / 2) * 720)

    @property
    def pt2(self):
        return int((self._x + self._w / 2) * 1280), int((self._y + self._h / 2) * 720)

    @property
    def center(self):
        return int(self._x * 1280), int(self._y * 720)

    @property
    def text(self):
        return self._info



markers = []


def on_detect_marker(marker_info):
    number = len(marker_info)
    markers.clear()
    for i in range(0, number):
        x, y, w, h, info = marker_info[i]
        markers.append(
            MarkerInfo(x, y, w, h, info)
        )

def sub_data_handler(angle_info):
    global list_of_data
    list_of_data = angle_info


if __name__ == "__main__":
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_blaster = ep_robot.blaster

    ep_vision = ep_robot.vision
    ep_camera = ep_robot.camera
    ep_gimbal = ep_robot.gimbal
    ep_blaster = ep_robot.blaster
    center_x = 1280 / 2
    center_y = 720 / 2
    ep_camera.start_video_stream(display=False)
    ep_gimbal.sub_angle(freq=10, callback=sub_data_handler)
    result = ep_vision.sub_detect_info(
        name="marker", callback=on_detect_marker
    )
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()
    init_time = time.time()
    count = 0
    time.sleep(1)

    p = (0.8/1.7)
    #period = (x / x)
    old_p = 0.1
    i = (p/(0.639/2))
    d = (p*(0.639/8))

    accumulate_err_x = 0
    accumulate_err_y = 0
    data_pith_yaw = []
    prev_time = time.time()
    prev_err_x = 0
    prev_err_y = 0
    while True:
        if len(markers) != 0:
            
            after_time = time.time()
            x, y = markers[-1].center

            err_x = (
                center_x - x
            )
            err_y = (
                center_y - y
            )
            accumulate_err_x += err_x*(after_time - prev_time)
            accumulate_err_y += err_y*(after_time - prev_time)

            if count >= 1:
                #ep_blaster.fire(fire_type= blaster.INFRARED_FIRE)
                
                speed_x = (
                    (p * err_x)
                    + d * ((err_x - prev_err_x ) / (after_time - prev_time))
                    + i * (accumulate_err_x)
                )
                speed_y = (
                    (p * err_y)
                    + d * ((err_y - prev_err_y) / (after_time - prev_time))
                    + i * (accumulate_err_y)
                )
                ep_gimbal.drive_speed(pitch_speed=speed_y, yaw_speed=-speed_x)
                data_pith_yaw.append(
                    list(list_of_data)
                    + [err_x, err_y, round(speed_x, 3), round(speed_y, 3), after_time - init_time]
                )

            count += 1
            
            
            prev_time = after_time
            prev_err_x = err_x
            prev_err_y = err_y
            time.sleep(0.001)
        else:
            ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)

        img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        for j in range(0, len(markers)):
            cv2.rectangle(img, markers[j].pt1, markers[j].pt2, (0, 255, 0))
            cv2.putText(
                img,
                markers[j].text,
                markers[j].center,
                cv2.FONT_HERSHEY_SIMPLEX,
                1.5,
                (0, 255, 0),
                3,
            )
        cv2.imshow("Markers", img)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()

    result = ep_vision.unsub_detect_info(name="marker")
    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()

    x_point = [i for i in range(len(data_pith_yaw))]

    y_point4 = [i[4] for i in data_pith_yaw]

    # y_point5 = [i[5] for i in data_pith_yaw]

    # y_point6 = [i[6] for i in data_pith_yaw]

    # y_point7 = [i[7] for i in data_pith_yaw]
    x_point = [i[8] for i in data_pith_yaw]

    plt.plot(x_point, y_point4)
    # plt.plot(x_point, y_point5)
    # plt.plot(x_point, y_point6)
    # plt.plot(x_point, y_point7)
    plt.legend(["e x", "e y", "u x", "u y"])
    plt.show()
    plt.savefig("graph.png")