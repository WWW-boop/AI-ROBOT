import cv2
import numpy as np

def convert_to_hsv_and_threshold(image):
    if len(image.shape) == 3:  # ตรวจสอบว่าภาพมีช่องสัญญาณ 3 ช่อง
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        return mask
    else:
        raise ValueError("Input image must be a 3-channel BGR image.")

def template_matching(frame, template):
    if len(frame.shape) == 3:
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        result = cv2.matchTemplate(frame_gray, template_gray, cv2.TM_CCOEFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
        return max_val, max_loc
    else:
        raise ValueError("Input image must be a 3-channel BGR image.")

cap = cv2.VideoCapture(0)

reference_img = cv2.imread(r'C:\Users\wikra\Desktop\robot ai\RoboMaster-SDK\examples\pic\Real_Coke.png')
if reference_img is not None and len(reference_img.shape) == 3:
    reference_img = cv2.resize(reference_img, (64, 128))
else:
    raise ValueError("Reference image must be a 3-channel BGR image.")

sizes = [(64, 128), (128, 256), (32, 64)]

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    if frame is not None and len(frame.shape) == 3:
        frame = cv2.resize(frame, (640, 480))
        frame_hsv_thresholded = convert_to_hsv_and_threshold(frame)
        
        best_match = 0
        best_location = None
        best_size = None

        for size in sizes:
            resized_template = cv2.resize(reference_img, size)
            max_val, top_left = template_matching(frame, resized_template)
            
            if max_val > best_match:
                best_match = max_val
                best_location = top_left
                best_size = size

        if best_match > 0.6:  # ปรับค่าความคล้ายที่ต้องการ
            bottom_right = (best_location[0] + best_size[0], best_location[1] + best_size[1])
            cv2.rectangle(frame, best_location, bottom_right, (0, 255, 0), 2)
            cv2.putText(frame, f'Similarity: {best_match:.2f}', (best_location[0], best_location[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        cv2.imshow('Coke Detection', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
