import cv2
import robomaster
from robomaster import robot
from robomaster import vision
import time
import matplotlib.pyplot as plt


# คลาสสำหรับเก็บข้อมูล marker(ป้าย)
class MarkerInfo:
    # ข้อมูลจุดตรงกลางป้าย ความกว้าง ความยาว ข้อมูลป้าย
    def __init__(self, x, y, w, h, info):
        self._x = x
        self._y = y
        self._w = w
        self._h = h
        self._info = info

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

    # ข้อมูลป้าย
    @property
    def text(self):
        return self._info

markers = []
list_of_data = [0, 0, 0]

# in case that there are many detected markers
def on_detect_marker(marker_info):
    number = len(marker_info)
    markers.clear()
    for i in range(0, number):
        x, y, w, h, info = marker_info[i]
        markers.append(
            MarkerInfo(x, y, w, h, info)
        )  # x and y w h is in a range of [0 1] relative to image's size

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

    # the image center constants
    center_x = 1280 / 2
    center_y = 720 / 2
    frame = 0

    ep_camera.start_video_stream(display=False)
    ep_gimbal.sub_angle(freq=50, callback=sub_data_handler)
    result = ep_vision.sub_detect_info(name="marker", callback=on_detect_marker)

    # หมุน gimbal กลับไปที่ center
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()

    count = 0
    time.sleep(1)

    # PID controller constants
    p = 0.1

    data_pitch_yaw = []

    # loop การทำงานของหุ่น
    previous_time = time.time()

    frame = 0 
    sw = True 

    while True:
        frame += 1 
        
        if frame % 300 == 0:
            sw = not sw
        if sw:
            center_x = 1280 * 0.25  # 1/4
        else:
            center_x = 1280 * 0.75  # 3/4x
        print(center_x)
        if len(markers) != 0:  # target found
            x, y = markers[-1].center  # x,y here in the pixel unit
            err_x = center_x - x  # err_x = image_center in x direction - current marker center in x direction
            err_y = center_y - y  # err_y = image_center in y direction - current marker center in y direction
            if count >= 1:
                # คำนวณความเร็วในการหมุน gimbal โดยใช้ PID
                speed_x = p * err_x
                speed_y = p * err_y
                # หมุน gimbal ตามความเร็วที่คำนวณมา
                print(f'this is speed_x : {speed_x}, this is speed_y :{speed_y}')
                ep_gimbal.drive_speed(pitch_speed=speed_y, yaw_speed=-speed_x)
                # เก็บค่ามุมของ gimbal, error x, error y, speed x, speed y, และ dt
                data_pitch_yaw.append(
                    list(list_of_data) + [err_x, err_y, round(speed_x, 3), round(-speed_y, 3),center_x,x]
                )
            count += 1
        else:
            # หมุนกลับ center
            ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
        # อ่านภาพ
        img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        # วาดสี่เหลี่ยมบนภาพในตำแหน่งที่เจอป้าย
        for marker in markers:
            cv2.rectangle(img, marker.pt1, marker.pt2, (0, 255, 0))
            cv2.putText(
                img,
                marker.text,
                marker.center,
                cv2.FONT_HERSHEY_SIMPLEX,
                1.5,
                (0, 255, 0),
                3,
            )
            cv2.putText(
                img,
                str(frame%300),
                (10,25),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.5,
                (0,255,0),
                3,
            )
        # แสดงภาพ
        cv2.imshow("Markers", img)
        # สำหรับออกจาก loop while
        if cv2.waitKey(1) & 0xFF == ord("e"):
            break
    cv2.destroyAllWindows()
    result = ep_vision.unsub_detect_info(name="marker")
    ep_camera.stop_video_stream()
    ep_robot.close()
    # plot error x, error y, speed x, speed y, and dt
    x_point = range(len(data_pitch_yaw))
    y_point4 = [i[4] for i in data_pitch_yaw]
    y_point5 = [i[5] for i in data_pitch_yaw]
    y_point6 = [i[6] for i in data_pitch_yaw]
    y_point7 = [i[7] for i in data_pitch_yaw]
    y_point8 = [i[8] for i in data_pitch_yaw]
    y_point9 = [i[9] for i in data_pitch_yaw]
    
    plt.plot(x_point,y_point8)
    plt.plot(x_point,y_point9)
    plt.legend(["center_x","x"])
    plt.show()
    


    plt.figure((10,5))
    plt.plot(center_x)
