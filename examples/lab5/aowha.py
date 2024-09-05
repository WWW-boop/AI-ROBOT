import cv2
import robomaster
from robomaster import robot
from robomaster import vision
from robomaster import blaster
import numpy as np
import time
from sklearn.metrics.pairwise import cosine_similarity

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

def detect_coke_can(frame, templates):
    # Convert RGB to HSV
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define the range of red color (adjust the range to match your Coke can)
    lower_hue1 = np.array([0, 0, 0])    # Lower range for red
    upper_hue1 = np.array([15, 255, 100])
    
    lower_hue2 = np.array([160, 50, 50])  # Red in the upper range
    upper_hue2 = np.array([180, 255, 255])

    # Threshold the HSV image to get only red colors
    mask1 = cv2.inRange(hsv_frame, lower_hue1, upper_hue1)
    mask2 = cv2.inRange(hsv_frame, lower_hue2, upper_hue2)
    
    # Combine both masks
    mask = mask1 | mask2
    
    # Apply morphological operations to reduce noise and improve detection
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Initialize a variable to store the best similarity score and bounding box
    best_similarity = 0
    best_box = None
    
    # Iterate over each contour to refine detection using cosine similarity
    if contours:
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w > 20 and h > 20:  # Adjust this filter to allow for smaller objects
                # Extract subregion of the frame
                subregion = frame[y:y+h, x:x+w]
                
                # Iterate over all template sizes to find the best match
                for template in templates:
                    # Resize the template to match the size of the subregion
                    resized_template = cv2.resize(template, (w, h))
                    
                    # Flatten both the resized template and the subregion
                    subregion_flat = subregion.flatten()
                    template_flat = resized_template.flatten()
                    
                    # Calculate cosine similarity between the subregion and the template
                    similarity = cosine_similarity([subregion_flat], [template_flat])[0, 0]
                    
                    # Update the best similarity and bounding box if needed
                    if similarity > best_similarity:
                        best_similarity = similarity
                        best_box = (x, y, w, h)
    
    # Draw the bounding box around the detected object with the highest similarity
    if best_box and best_similarity > 0.5:  # Lower similarity threshold for distant objects
        x, y, w, h = best_box
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, f'Similarity: {best_similarity:.2f}', (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    return frame

def process_image(frame, templates):
    # Detect Coke-can using color and cosine similarity with multiple templates
    result_frame = detect_coke_can(frame, templates)
    return result_frame

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
    ep_blaster = ep_robot.blaster

    # the image center constants
    center_x = 1280 / 2
    center_y = 720 / 2

    ep_camera.start_video_stream(display=False)
    ep_gimbal.sub_angle(freq=50, callback=sub_data_handler)

    # หมุน gimbal กลับไปที่ center
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()

    count = 0
    time.sleep(1)

    # PID controller constants
    p = -0.6
    i = 0
    d = 0

    data_pith_yaw = []

    # Load templates
    templates = [
        cv2.imread(r'C:\Users\lataeq\AI-ROBOT\examples\pic\coke-1block.jpg'),
        cv2.imread(r'C:\Users\lataeq\AI-ROBOT\examples\pic\coke-1block.jpg'),
        cv2.imread(r'C:\Users\lataeq\AI-ROBOT\examples\pic\coke-4block.jpg')
    ]

    # loop การทำงานของหุ่น
    while True:
        # อ่านภาพ
        img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)

        # Detect Coke can
        result_frame = process_image(img, templates)

        # วาดสี่เหลี่ยมบนภาพในตำแหน่งที่เจอป้าย
        for j in range(0, len(markers)):
            cv2.rectangle(result_frame, markers[j].pt1, markers[j].pt2, (0, 255, 0))
            cv2.putText(
                result_frame,
                markers[j].text,
                markers[j].center,
                cv2.FONT_HERSHEY_SIMPLEX,
                1.5,
                (0, 255, 0),
                3,
            )
        # แสดงภาพ
        cv2.imshow("Markers", result_frame)

        if len(markers) != 0:  # target found
            after_time = time.time()
            x, y = markers[-1].center  # x,y here in the pixel unit

            err_x = (center_x - x)  # err_x = image_center in x direction - current marker center in x direction
            err_y = (center_y - y)  # err_y = image_center in y direction - current marker center in y direction

            if count >= 1:
                # คำนวณความเร็วในการหมุน gimbal โดยใช้ PID
                speed_x = ((p * err_x))
                speed_y = ((p * err_y))

                # หมุน gimbal ตามความเร็วที่คำนวณมาก
                ep_gimbal.drive_speed(pitch_speed=-speed_y, yaw_speed=speed_x)

                # เก็บค่ามุมของ gimbal, error x, error y, speed x, speed y
                data_pith_yaw.append(list(list_of_data)+ [err_x, err_y, round(speed_x, 3), round(speed_y, 3)])

            count += 1

        else:
            # หมุนกลับ center
            ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)

        # สำหรับออกจาก loop while
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()

    result = ep_vision.unsub_detect_info(name="marker")
    ep_camera.stop_video_stream()
    ep_robot.close()