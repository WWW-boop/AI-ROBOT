import cv2
import numpy as np

def detect_coke_can(frame, lower_hue1, upper_hue1, lower_hue2, upper_hue2):
    # Convert RGB to HSV
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
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
    
    # If contours are found, draw bounding boxes
    if contours:
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            # Filter out small contours that may be noise
            if w > 50 and h > 50:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    
    return frame

# For image input
def process_image(image_path):
    # Load the image from file
    frame = cv2.imread(image_path)
    
    if frame is None:
        print(f"Error: could not load image from {image_path}")
        return
    
    # Define the range of red color (you can adjust this range to match your specific Coke can)
    # Example hue ranges for the Coke can; adjust these values as needed based on your analysis
    lower_hue1 = np.array([0, 50, 50])    # Lower range for red
    upper_hue1 = np.array([10, 255, 255]) # Upper range for red

    lower_hue2 = np.array([160, 50, 50])  # Red in the upper range
    upper_hue2 = np.array([180, 255, 255])
    
    # Detect Coke-can
    result_frame = detect_coke_can(frame, lower_hue1, upper_hue1, lower_hue2, upper_hue2)
    
    # Save the result
    result_path = 'detected_coke_can_output.png'
    cv2.imwrite(result_path, result_frame)
    
    # Display the result
    cv2.imshow('Detection Result', result_frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Example usage
if __name__ == "__main__":
    process_image(r'RoboMaster-SDK\examples\pic\coke-1block.jpg')
