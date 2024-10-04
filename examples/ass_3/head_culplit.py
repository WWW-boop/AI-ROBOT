import cv2
import numpy as np
import time
from robomaster import robot, camera

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

        if 50 < cv2.contourArea(bottle_contour_max) < 5000:  # ขนาดขั้นต่ำและสูงสุด
            approx = cv2.approxPolyDP(bottle_contour_max, 0.02 * cv2.arcLength(bottle_contour_max, True), True)
            if len(approx) > 4:  # ตรวจสอบจำนวนด้านของรูปร่าง
                x, y, w, h = cv2.boundingRect(bottle_contour_max)
                aspect_ratio = float(w) / h
                if 0.8 < aspect_ratio < 1.2:  # ตรวจสอบอัตราส่วน
                    cv2.drawContours(img, [bottle_contour_max], -1, (0, 255, 0), 2)
                    cv2.putText(img, "Bottle Detected", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

    return img


# ฟังก์ชันหลัก
def main():
    choice = input("คุณต้องการใช้งานภาพจากไฟล์หรือกล้อง RoboMaster? (file/robot): ").strip().lower()

    if choice == "file":
        # อ่านรูปภาพจากไฟล์
        img_path = "C:\\Users\\User\\Documents\\GitHub\\AI-ROBOT\\examples\\lab4\\agent.jpg"  # เปลี่ยน path นี้ให้ถูกต้อง
        img = cv2.imread(img_path)

        if img is None:
            print("Error: ไม่สามารถเปิดรูปภาพได้!")
            return

        # แปลงรูปภาพเป็น HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # เรียกใช้ฟังก์ชันตรวจจับฝาสีฟ้า
        result_img = blue_head_culprit(hsv, img)

        # แสดงผลลัพธ์
        cv2.imshow("Result", result_img)
        cv2.waitKey(0)  # รอการกดปุ่มเพื่อปิดหน้าต่างแสดงผล
        cv2.destroyAllWindows()

    elif choice == "robot":
        # เชื่อมต่อ RoboMaster
        ep_robot = robot.Robot()
        ep_robot.initialize(conn_type="ap")

        ep_camera = ep_robot.camera
        ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

        try:
            while True:
                # รับภาพจากกล้อง RoboMaster
                img = ep_camera.read_cv2_image(strategy="newest")

                # แปลงเป็น HSV
                hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

                # เรียกใช้ฟังก์ชันตรวจจับฝาสีฟ้า
                result_img = blue_head_culprit(hsv, img)

                # แสดงผลลัพธ์
                cv2.imshow("RoboMaster Detection", result_img)

                # กด 'q' เพื่อหยุด
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            # ปิดวิดีโอสตรีมและปิดการเชื่อมต่อ RoboMaster
            ep_camera.stop_video_stream()
            ep_robot.close()
            cv2.destroyAllWindows()

    else:
        print("กรุณาเลือก 'file' หรือ 'robot'")

if __name__ == "__main__":
    main()
