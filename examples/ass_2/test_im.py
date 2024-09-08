import cv2
import numpy as np

# Load the image
image = cv2.imread(r'D:\study\241-251\AI-ROBOT\examples\ass_2\ass23.jpg')

# Step 1: Convert image from BGR to HSV
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Step 2: Define lower and upper range for yellow color
# Adjust these values according to the object's shade of yellow
lower_yellow = np.array([32, 180, 0])  # Lower range of yellow (Hue, Saturation, Value)
upper_yellow = np.array([50, 255, 163])  # Upper range of yellow

# Step 3: Create a mask to isolate the yellow object
mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

# Step 4: Apply the mask to the original image
result = cv2.bitwise_and(image, image, mask=mask)

# Step 5: Convert masked result to grayscale and apply threshold
gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)

# Step 6: Find contours from the thresholded image
contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Step 7: Filter and draw contours
min_area = 100  # Adjust this based on your object size
for contour in contours:
    area = cv2.contourArea(contour)
    if area > min_area:
        # Draw contour for the yellow object
        cv2.drawContours(image, [contour], -1, (0, 255, 0), 2)  # Green contour for the object

# Step 8: Display the result
cv2.imshow('Yellow Object Detection', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
