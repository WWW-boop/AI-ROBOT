import cv2
import numpy as np
import time
from robomaster import robot, blaster, camera

# PID control parameters
p = 0.2
i = 0.01
d = 0.05

# Smoothing factor for bounding box and gimbal speed
alpha = 0.7  # Smoothing factor for bounding box
max_speed = 200  # Maximum speed for gimbal movement

# Function to smooth bounding box coordinates or gimbal speed
def smooth_bbox(smooth_val, new_val, alpha):
    return smooth_val * (1 - alpha) + new_val * alpha

# Bottle detection function (blue cap detection)
def blue_head_culprit(hsv, img):
    lower_hue_bottle = np.array([99, 122, 88])
    upper_hue_bottle = np.array([111, 246, 255])

    bottle_mask = cv2.inRange(hsv, lower_hue_bottle, upper_hue_bottle)

    kernel = np.ones((5, 5), np.uint8)
    bottle_mask = cv2.morphologyEx(bottle_mask, cv2.MORPH_CLOSE, kernel)
    bottle_mask = cv2.morphologyEx(bottle_mask, cv2.MORPH_OPEN, kernel)
    bottle_mask = cv2.GaussianBlur(bottle_mask, (5, 5), 0)

    bottle_contours, _ = cv2.findContours(bottle_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(bottle_contours) > 0:
        bottle_contour_max = max(bottle_contours, key=cv2.contourArea)

        if 50 < cv2.contourArea(bottle_contour_max) < 5000:  # Minimum and maximum size
            approx = cv2.approxPolyDP(bottle_contour_max, 0.02 * cv2.arcLength(bottle_contour_max, True), True)
            if len(approx) > 4:  # Check the number of sidesq
                x, y, w, h = cv2.boundingRect(bottle_contour_max)
                aspect_ratio = float(w) / h
                if 0.8 < aspect_ratio < 1.2:  # Check aspect ratio
                    # cv2.drawContours(img, [bottle_contour_max], -1, (0, 255, 0), 2)
                    # cv2.putText(img, "Bottle Detected", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                    return x, y, w, h  # Return bounding box coordinates
    return None


def body_acrylic_detect(img):
    # กำหนดขอบเขตของ ROI
    x_start, y_start, x_end, y_end = 560, 470, 740, 600
    roi = img[y_start:y_end, x_start:x_end]  # ตัดภาพเฉพาะส่วนที่กำหนด
    # cv2.imshow("roi", roi)

    # การเบลอเฉพาะส่วน ROI บน CPU
    blurred_roi = cv2.GaussianBlur(roi, (5, 5), 0)
    
    # แปลงเป็นสีเทา
    gray_roi = cv2.cvtColor(blurred_roi, cv2.COLOR_BGR2GRAY)

    # ทำการตรวจจับขอบ (Edge detection) เฉพาะส่วน ROI
    edges_roi = cv2.Canny(gray_roi, 44, 138)
    # cv2.imshow("edroi", edges_roi)

    # หาขอบเขตของวัตถุ
    contours, _ = cv2.findContours(edges_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 1:  # กรอง contour ขนาดเล็กออก
            if len(cnt) >= 5:  # ฟังก์ชัน fitEllipse ต้องการ contour ที่มีอย่างน้อย 5 จุด
                ellipse = cv2.fitEllipse(cnt)  # ประมาณวงรีจาก contour
                cv2.ellipse(roi, ellipse, (0, 255, 0), 2)  # วาดวงรีที่ตรวจพบ
                
    # นำ ROI ที่ปรับปรุงแล้วกลับไปที่ภาพต้นฉบับ
    img[y_start:y_end, x_start:x_end] = roi
    return img

def gai_detect(hsv, img):
    x_start, y_start, x_end, y_end = 200, 320, 960, 670  #xsrart 200 or 320
    roi = img[y_start:y_end, x_start:x_end]
    hsv_roi = hsv[y_start:y_end, x_start:x_end]

    lower_hue_gai = np.array([29, 235, 85])
    upper_hue_gai = np.array([37, 255, 255])
    gai_mask = cv2.inRange(hsv_roi, lower_hue_gai, upper_hue_gai)
    
    kernel = np.ones((5, 5), np.uint8)
    gai_mask = cv2.GaussianBlur(gai_mask, (9, 9), 0)
    gai_mask = cv2.morphologyEx(gai_mask, cv2.MORPH_CLOSE, kernel)
    gai_mask = cv2.morphologyEx(gai_mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(gai_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if 10 > area > 100:  #ขนาด
            x, y, w, h = cv2.boundingRect(cnt)
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)

            if len(approx) > 5: #ค่าความเหมือน
                cv2.drawContours(roi, [approx], -1, (0, 255, 0), 3)
                cv2.putText(roi, "Chicken Detected", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

    return img

def main():
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="rndis")
    ep_camera = ep_robot.camera
    ep_gimbal = ep_robot.gimbal
    ep_blaster = ep_robot.blaster
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    ep_gimbal.sub_angle(freq=10)
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()

    time.sleep(1)  

    detected = None
    detection_start_time = None  
    detection_duration = 0  
    frame_count = 0  # เพิ่มตัวนับจำนวนเฟรม
    last_recenter_time = time.time()  # เวลาล่าสุดที่รีเซ็นเตอร์ Gimbal

    center_x = 1280 / 2
    center_y = 720 / 2

    while True:
        frame_count += 1  # เพิ่มค่าตัวนับเฟรม
        if frame_count % 5 != 0:  # ประมวลผลเฉพาะทุกๆ 5 เฟรม
            continue

        img = ep_camera.read_cv2_image(strategy="newest")
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        bottle_bbox = blue_head_culprit(hsv, img)

        if bottle_bbox:
            x, y, w, h = bottle_bbox

            error_x = (x + w / 2) - center_x
            error_y = (y + h / 2) - center_y

            yaw_speed = p * error_x
            pitch_speed = p * error_y

            yaw_speed = np.clip(yaw_speed, -max_speed, max_speed)
            pitch_speed = np.clip(pitch_speed, -max_speed, max_speed)

            ep_gimbal.drive_speed(pitch_speed=-pitch_speed, yaw_speed=yaw_speed)

            if detection_start_time is None:
                detection_start_time = time.time()  
            detection_duration = time.time() - detection_start_time  

            if detection_duration > 3 :
                
                ep_blaster.fire(fire_type=blaster.WATER_FIRE, times=5)
                break

        else:
            detection_start_time = None
            detection_duration = 0

            # รีเซ็นเตอร์ Gimbal เมื่อไม่มีการตรวจพบขวด
            if time.time() - last_recenter_time > 2:  # หน่วงเวลา 2 วินาที
                ep_gimbal.recenter(pitch_speed=100, yaw_speed=100).wait_for_completed()
                last_recenter_time = time.time()  # อัพเดทเวลาล่าสุดที่รีเซ็นเตอร์
                break
        img = body_acrylic_detect(img)
        img = gai_detect(hsv, img)

        cv2.imshow("Detection", img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    ep_camera.stop_video_stream()
    ep_robot.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
