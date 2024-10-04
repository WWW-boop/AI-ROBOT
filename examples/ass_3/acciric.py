import cv2
import numpy as np
import time
from robomaster import robot, camera

# ฟังก์ชันตรวจจับวัตถุโปร่งใส (อะคริลิก)
def detect_transparent_object(img):
    # แปลงรูปเป็น Gaussian Kernel ก่อนการตรวจจับ
    rows, cols = img.shape[:2]

    # สร้าง Mask เพื่อลดการตรวจจับส่วนบนของภาพ
    mask = np.zeros((rows, cols), dtype=np.uint8)
    mask[int(rows * 0.3):, :] = 255  # กำหนดให้ใช้เฉพาะส่วนล่าง 70% ของภาพ

    # กำหนด Gaussian kernel สำหรับการปรับแสง
    kernel_x = cv2.getGaussianKernel(cols, 270)
    kernel_y = cv2.getGaussianKernel(rows, 200)
    kernel = kernel_y * kernel_x.T
    kernel = kernel / np.linalg.norm(kernel)

    # คูณ mask กับรูป
    masked_img = cv2.bitwise_and(img, img, mask=mask)
    
    # แปลงเป็นระดับสีเทา
    gray = cv2.cvtColor(masked_img, cv2.COLOR_BGR2GRAY)
    cv2.imshow("dasdad",gray)
    # ตรวจจับขอบด้วย Canny Edge Detection
    edges = cv2.Canny(gray, 85, 346)
    cv2.imshow("dadadf",edges)
    # ตรวจจับแสงสะท้อนในวัตถุโปร่งใส
    _, reflection = cv2.threshold(gray, 90, 255, cv2.THRESH_BINARY)
    cv2.imshow("dada",reflection)
    # รวมผลลัพธ์จาก edge detection และการตรวจจับแสงสะท้อน
    combined = cv2.bitwise_xor(edges, reflection)

    # ใช้ Morphological Transformations เพื่อลด noise
    kernel = np.ones((1, 1), np.uint8)
    cv2.imshow("dad",kernel)
    combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)
    combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, kernel)

    # ใช้ cv2.dilate เพื่อขยายเส้นขอบ
    combined = cv2.dilate(combined, kernel, iterations=1)

    # ค้นหาขอบเขตวัตถุ
    contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.imshow("dadad",combined)
    if contours:
        # กรองขนาดของวัตถุที่ตรวจจับ
        contours = [c for c in contours if 100 < cv2.contourArea(c) < 5000]

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            epsilon = 0.01 * cv2.arcLength(largest_contour, True)
            approx = cv2.approxPolyDP(largest_contour, epsilon, True)
            cv2.drawContours(img, [approx], -1, (0, 255, 0), 2)

            # เพิ่มข้อความบอกตำแหน่งวัตถุโปร่งใส
            x, y, w, h = cv2.boundingRect(approx)
            cv2.putText(img, "Transparent Object Detected", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    return img




# ฟังก์ชันหลัก
def main():
    choice = input("คุณต้องการใช้งานภาพจากไฟล์หรือกล้อง RoboMaster? (file/ro): ").strip().lower()

    if choice == "file":
        # อ่านรูปภาพจากไฟล์
        img_path = "C:\\Users\\User\\Documents\\GitHub\\AI-ROBOT\\examples\\lab4\\blackarci.jpg"  # เปลี่ยน path นี้ให้ถูกต้อง
        img = cv2.imread(img_path)

        if img is None:
            print("Error: ไม่สามารถเปิดรูปภาพได้!")
            return

        # เรียกใช้ฟังก์ชันตรวจจับวัตถุโปร่งใส
        result_img = detect_transparent_object(img)

        # แสดงผลลัพธ์
        cv2.imshow("Result", result_img)
        cv2.waitKey(0)  # รอการกดปุ่มเพื่อปิดหน้าต่างแสดงผล
        cv2.destroyAllWindows()

    elif choice == "ro":
        # เชื่อมต่อ RoboMaster
        ep_robot = robot.Robot()
        ep_robot.initialize(conn_type="ap")

        ep_camera = ep_robot.camera
        ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

        try:
            while True:
                # รับภาพจากกล้อง RoboMaster
                img = ep_camera.read_cv2_image(strategy="newest")

                # เรียกใช้ฟังก์ชันตรวจจับวัตถุโปร่งใส
                result_img = detect_transparent_object(img)

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
        print("กรุณาเลือก 'file' หรือ 'ro'")

if __name__ == "__main__":
    main()
