import cv2
import numpy as np

def nothing(x):
    pass

def pick_color(image):
    # สร้างหน้าต่างที่ปรับขนาดได้
    cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    
    # สร้าง Trackbars สำหรับปรับค่า HSV
    cv2.createTrackbar('H Lower', 'image', 0, 179, nothing)
    cv2.createTrackbar('S Lower', 'image', 0, 255, nothing)
    cv2.createTrackbar('V Lower', 'image', 0, 255, nothing)
    cv2.createTrackbar('H Upper', 'image', 179, 179, nothing)
    cv2.createTrackbar('S Upper', 'image', 255, 255, nothing)
    cv2.createTrackbar('V Upper', 'image', 255, 255, nothing)

    while True:
        # อ่านค่าจาก Trackbars
        h_lower = cv2.getTrackbarPos('H Lower', 'image')
        s_lower = cv2.getTrackbarPos('S Lower', 'image')
        v_lower = cv2.getTrackbarPos('V Lower', 'image')
        h_upper = cv2.getTrackbarPos('H Upper', 'image')
        s_upper = cv2.getTrackbarPos('S Upper', 'image')
        v_upper = cv2.getTrackbarPos('V Upper', 'image')

        # สร้างขอบเขต HSV
        lower_bound = np.array([h_lower, s_lower, v_lower])
        upper_bound = np.array([h_upper, s_upper, v_upper])

        # แปลงภาพจาก BGR เป็น HSV
        hsv_frame = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # สร้าง Mask โดยใช้ค่า HSV
        mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)

        # ใช้ Morphological Operations เพื่อลด noise และขยายขอบเขตการตรวจจับ
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # รวม Mask กับภาพต้นฉบับ
        result = cv2.bitwise_and(image, image, mask=mask)

        # แสดงผลภาพที่ตรวจจับ
        cv2.imshow('image', result)

        # กดปุ่ม Escape เพื่อออกจากลูป
        key = cv2.waitKey(1)
        if key == 27:  # ปุ่ม Escape
            break

    # ปิดหน้าต่างทั้งหมด
    cv2.destroyAllWindows()

# โหลดและปรับขนาดภาพ
image_path = r'D:\study\241-251\AI-ROBOT\examples\ass_2\IMG_5910.jpg'
image = cv2.imread(image_path)

if image is None:
    print(f"Error: ไม่สามารถโหลดภาพจาก {image_path}")
else:
    # ปรับขนาดภาพให้พอดีกับหน้าจอแสดงผล (ปรับตามความต้องการของคุณ)
    resized_image = cv2.resize(image, (800, 600))  # ขนาดปรับได้ตามต้องการ

    # เรียกใช้ฟังก์ชัน pick_color เพื่อตรวจจับสี
    pick_color(resized_image)
