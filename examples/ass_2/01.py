import cv2
import numpy as np
from sklearn.metrics.pairwise import cosine_similarity
from robomaster import robot, vision, blaster
import time

# Function to detect water bottle using HSV color matching and smoothing
def detect_water_bottle(frame, templates, prev_box, alpha=0.2):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define HSV range for blue color in the bottle label
    lower_blue = np.array([100, 150, 50])  # Modify values as needed
    upper_blue = np.array([140, 255, 255])
    
    # Create mask for blue color detection
    mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)

    # Apply morphological transformations to clean up the mask
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    # Find contours based on the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    best_similarity = 0
    best_box = None
    
    if contours:
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w > 20 and h > 20:  # Filter small contours
                subregion = frame[y:y+h, x:x+w]
                
                # Compare each subregion with the provided template
                for template in templates:
                    resized_template = cv2.resize(template, (w, h))
                    subregion_flat = subregion.flatten()
                    template_flat = resized_template.flatten()
                    similarity = cosine_similarity([subregion_flat], [template_flat])[0, 0]
                    
                    if similarity > best_similarity:
                        best_similarity = similarity
                        best_box = (x, y-60, w, h*5)
    
    # Apply smoothing to the bounding box
    if best_box and best_similarity > 0.5:
        x, y, w, h = best_box
        
        if prev_box is not None:
            prev_x, prev_y, prev_w, prev_h = prev_box
            
            # Smooth the position and size of the bounding box
            x = int(alpha * x + (1 - alpha) * prev_x)
            y = int(alpha * y + (1 - alpha) * prev_y)
            size_alpha = 0.1  # Use a smaller alpha for size changes
            w = int(size_alpha * w + (1 - size_alpha) * prev_w)
            h = int(size_alpha * h + (1 - size_alpha) * prev_h)
        
        # Draw the bounding box on the frame
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, f'Similarity: {best_similarity:.2f}', (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        return frame, (x, y, w, h)
    else:
        return frame, prev_box

def process_image(frame, templates, prev_box):
    result_frame, updated_box = detect_water_bottle(frame, templates, prev_box)
    return result_frame, updated_box



def sub_data_handler(angle_info):
    global list_of_data
    list_of_data = angle_info

# Main function
if __name__ == "__main__":
    list_of_data = []
    
    # Load templates
    templates = [
        cv2.imread(r'D:\study\241-251\AI-ROBOT\examples\ass_2\IMG_5907.jpg'),
        cv2.imread(r'D:\study\241-251\AI-ROBOT\examples\ass_2\IMG_5908.jpg'),
        cv2.imread(r'D:\study\241-251\AI-ROBOT\examples\ass_2\IMG_5909.jpg')  # Use the uploaded image for template matching
    ]

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_gimbal = ep_robot.gimbal
    ep_gimbal.recenter().wait_for_completed()
    ep_camera.start_video_stream(display=False)

    center_x = 1280 / 2
    center_y = 720 / 2

    # PID controller constants
    p = -0.607
    i = 0
    d = -0.00135

    accumulate_err_x = 0
    accumulate_err_y = 0
    data_pith_yaw = []
    prev_box = None  # Initial previous box
    alpha = 0.2  # Smoothing factor
    count = 0

    while True:
        after_time = time.time()
        img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        
        # Detect water bottle and smooth bounding box
        result_frame, prev_box = process_image(img, templates, prev_box)

        
        # Show the frame with detection
        cv2.imshow("Water Bottle Detection", result_frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()
