import cv2
import numpy as np
from sklearn.metrics.pairwise import cosine_similarity
from robomaster import robot, camera

def detect_coke_can(frame, templates):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    lower_hue1 = np.array([0, 0, 0])
    upper_hue1 = np.array([15, 255, 100])
    
    lower_hue2 = np.array([160, 50, 50])
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
    
    if best_box and best_similarity > 0.5:
        x, y, w, h = best_box
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, f'Similarity: {best_similarity:.2f}', (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    return frame

def stream_video_with_detection(templates):
    # Initialize RoboMaster robot
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")  # Connect via Wi-Fi
    
    # Start the video stream
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    
    while True:
        # Get a frame from the camera
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        
        if frame is not None:
            # Detect Coke-can in the current frame
            result_frame = detect_coke_can(frame, templates)
            
            # Display the frame with the detected object
            cv2.imshow('Coke Can Detection', result_frame)
            
            # Press 'q' to exit the loop
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    # Stop the video stream and release resources
    ep_camera.stop_video_stream()
    ep_robot.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # Load the templates
    templates = [cv2.imread(r'RoboMaster-SDK\examples\pic\coke-1block.jpg'), 
                 cv2.imread(r'RoboMaster-SDK\examples\pic\coke-3block.jpg'), 
                 cv2.imread(r'RoboMaster-SDK\examples\pic\coke-4block.jpg')]

    # Start streaming video with Coke-can detection
    stream_video_with_detection(templates)
