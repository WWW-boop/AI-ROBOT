import cv2
import numpy as np
from sklearn.metrics.pairwise import cosine_similarity
import robomaster
from robomaster import robot
from robomaster import vision
from robomaster import blaster

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

# For image input
def process_image(image_path, template_paths):
    # Load the image
    frame = cv2.imread(image_path)
    
    # Load all templates in different sizes
    templates = [cv2.imread(template_path) for template_path in template_paths]
    
    if frame is None or any(template is None for template in templates):
        print(f"Error: could not load image or one of the templates.")
        return
    
    # Detect Coke-can using color and cosine similarity with multiple templates
    result_frame = detect_coke_can(frame, templates)
    
    # Save the result
    result_path = 'detected_coke_can_output.png'
    cv2.imwrite(result_path, result_frame)
    
    # Display the result
    cv2.imshow('Detection Result', result_frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Example usage
if __name__ == "__main__":
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_vision = ep_robot.vision
    ep_camera = ep_robot.camera
    ep_gimbal = ep_robot.gimbal
    ep_blaster = ep_robot.blaster

    
    process_image(r'RoboMaster-SDK\examples\pic\coke-1block.jpg', 
                  [r'RoboMaster-SDK\examples\pic\coke-1block.jpg', r'RoboMaster-SDK\examples\pic\coke-3block.jpg', r'RoboMaster-SDK\examples\pic\coke-4block.jpg'])