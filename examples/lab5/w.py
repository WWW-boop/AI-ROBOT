import cv2
import numpy as np
from sklearn.metrics.pairwise import cosine_similarity
from robomaster import robot

# คลาสสำหรับเก็บข้อมูล marker(ป้าย)
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

# ฟังก์ชันตรวจจับ Coke can ด้วยการ smoothing
def detect_coke_can(frame, templates, prev_box, alpha=0.2):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    lower_hue1 = np.array([0, 120, 70])
    upper_hue1 = np.array([10, 255, 255])
    lower_hue2 = np.array([170, 120, 70])
    upper_hue2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv_frame, lower_hue1, upper_hue1)
    mask2 = cv2.inRange(hsv_frame, lower_hue2, upper_hue2)
    mask = mask1 | mask2

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    best_similarity = 0
    best_box = None
    
    if contours:
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w > 20 and h > 20:
                subregion = frame[y:y+h, x:x+w]
                
                for template in templates:
                    resized_template = cv2.resize(template, (w, h))
                    subregion_flat = subregion.flatten()
                    template_flat = resized_template.flatten()
                    similarity = cosine_similarity([subregion_flat], [template_flat])[0, 0]
                    
                    if similarity > best_similarity:
                        best_similarity = similarity
                        best_box = (x, y, w, h)
    
    # Smoothing bounding box ด้วย weighted average
    if best_box and best_similarity > 0.5:
        x, y, w, h = best_box
        
        if prev_box is not None:
            prev_x, prev_y, prev_w, prev_h = prev_box
            x = int(alpha * x + (1 - alpha) * prev_x)
            y = int(alpha * y + (1 - alpha) * prev_y)
            w = int(alpha * w + (1 - alpha) * prev_w)
            h = int(alpha * h + (1 - alpha) * prev_h)
        
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, f'Similarity: {best_similarity:.2f}', (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        return frame, (x, y, w, h)
    else:
        return frame, prev_box

def process_image(frame, templates, prev_box):
    result_frame, updated_box = detect_coke_can(frame, templates, prev_box)
    return result_frame, updated_box

# ฟังก์ชันหลัก
if __name__ == "__main__":
    # Load templates
    templates = [
        cv2.imread(r'RoboMaster-SDK\examples\pic\coke-1block.jpg'),
        cv2.imread(r'RoboMaster-SDK\examples\pic\coke-3block.jpg'),
        cv2.imread(r'RoboMaster-SDK\examples\pic\coke-4block.jpg')
    ]

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False)

    prev_box = None  # เก็บ bounding box จากเฟรมก่อนหน้า
    alpha = 0.2  # ปัจจัยการ smooth

    while True:
        img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        
        # ตรวจจับและทำ smoothing bounding box
        result_frame, prev_box = process_image(img, templates, prev_box)
        
        # แสดงผล
        cv2.imshow("Coke Can Detection", result_frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()
