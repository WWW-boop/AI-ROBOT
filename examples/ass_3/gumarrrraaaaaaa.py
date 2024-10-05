import cv2
import numpy as np

def process_image(image_path):
    # โหลดรูปภาพ
    image = cv2.imread(image_path)

    # สร้าง blank mask ที่มีขนาดเท่ากับรูปภาพ
    blank = np.zeros(image.shape, dtype='uint8')

    # แปลงเป็นภาพโทนสีเทา
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # วาดสี่เหลี่ยมใน blank mask
    cv2.rectangle(blank, (556, 429), (623, 597), (1, 1, 1), -1)

    # ใช้ GaussianBlur เพื่อลด noise
    blurred = cv2.GaussianBlur(gray, (3, 3), 0)

    # ใช้ Canny edge detection
    edges = cv2.Canny(blurred, 65, 225)

    # หาขอบเขต (contour) ของรูปร่าง
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # สร้างหน้ากาก (mask) ว่าง ๆ ที่มีขนาดเท่ากับรูปภาพ
    mask = np.zeros_like(image)

    # วาด contour บนหน้ากาก
    cv2.drawContours(mask, contours, -1, (255, 255, 255), thickness=cv2.FILLED)

    # ใช้ GaussianBlur กับภาพเทา
    blur = cv2.GaussianBlur(gray, (99, 99), cv2.BORDER_DEFAULT)

    # คัดลอกภาพต้นฉบับไปยัง blank ตามหน้ากากที่ทำไว้
    result1 = cv2.copyTo(image, blank, blur)

    # ใช้หน้ากากตัดเฉพาะส่วนที่เป็นคน
    cropped_image = cv2.bitwise_and(image, mask)

    # ทำการรวมภาพที่ได้
    plzgod = cv2.bitwise_and(cropped_image, result1)

    # แสดงผลลัพธ์
    cv2.imshow("Processed Image", plzgod)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# เรียกใช้ฟังก์ชัน
process_image("C:\\Users\\User\\Documents\\GitHub\\AI-ROBOT\\examples\\lab4\\blackarci.jpg")
