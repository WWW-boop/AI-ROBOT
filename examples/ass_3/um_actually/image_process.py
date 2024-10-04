import cv2
import numpy as np

# Callback function for the trackbars (needed by createTrackbar)
def nothing(x):
    pass

# Load the image
image = cv2.imread(r"C:\Users\lataeq\AI-ROBOT\examples\lab4\agent.jpg")
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Create a window
cv2.namedWindow('Canny Edge Detection')

# Create trackbars for threshold1, threshold2, and GaussianBlur kernel size
cv2.createTrackbar('Threshold1', 'Canny Edge Detection', 100, 500, nothing)
cv2.createTrackbar('Threshold2', 'Canny Edge Detection', 200, 500, nothing)
cv2.createTrackbar('Blur', 'Canny Edge Detection', 5, 20, nothing)

while True:
    # Get current positions of the trackbars
    t1 = cv2.getTrackbarPos('Threshold1', 'Canny Edge Detection')
    t2 = cv2.getTrackbarPos('Threshold2', 'Canny Edge Detection')
    blur_size = cv2.getTrackbarPos('Blur', 'Canny Edge Detection')

    # Ensure blur_size is odd for GaussianBlur (it must be odd)
    if blur_size % 2 == 0:
        blur_size += 1

    # Apply GaussianBlur to reduce noise
    blurred_image = cv2.GaussianBlur(gray_image, (blur_size, blur_size), 0)

    # Apply Canny Edge Detection with the current thresholds
    edges = cv2.Canny(blurred_image, t1, t2)

    # Display the result
    cv2.imshow('Canny Edge Detection', edges)

    # Break the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up windows
cv2.destroyAllWindows()
