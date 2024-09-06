import cv2
import numpy as np

def detect_coke_can_sliding_window(frame, templates, prev_box, window_size=(150, 150), stride=50, alpha=0.2, similarity_threshold=0.5):
    """
    Detect the Coke can using a sliding window approach.

    Args:
        frame: The current frame of the video.
        templates: A list of template images of the Coke can.
        prev_box: The previous bounding box of the Coke can.
        window_size: The size of the sliding window.
        stride: The stride of the sliding window.
        alpha: The learning rate for the template update.
        similarity_threshold: The threshold for the similarity score.

    Returns:
        The detected bounding box of the Coke can.
    """
    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the HSV color range for the Coke can
    lower_coke = np.array([0, 100, 100])
    upper_coke = np.array([10, 255, 255])

    # Threshold the HSV image to get a binary mask
    mask = cv2.inRange(hsv, lower_coke, upper_coke)

    # Apply morphological operations to the binary mask
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Initialize the detected bounding box
    detected_box = None

    # Iterate over the sliding window
    for x in range(0, frame.shape[1] - window_size[0], stride):
        for y in range(0, frame.shape[0] - window_size[1], stride):
            # Extract the current window
            window = frame[y:y + window_size[1], x:x + window_size[0]]

            # Compute the similarity score between the window and the templates
            scores = []
            for template in templates:
                score = cv2.matchTemplate(window, template, cv2.TM_CCOEFF_NORMED)
                scores.append(score)

            # Compute the average similarity score
            avg_score = np.mean(scores)

            # Check if the similarity score is above the threshold
            if avg_score > similarity_threshold:
                # Update the detected bounding box
                detected_box = (x, y, window_size[0], window_size[1])

                # Update the template using the current window
                template = cv2.resize(window, (template.shape[1], template.shape[0]))
                templates.append(template)

                # Break out of the loop
                break

    # Draw a larger bounding box to fully enclose the Coke can
    if detected_box is not None:
        x, y, w, h = detected_box
        x -= 10
        y -= 10
        w += 20
        h += 20

        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    return detected_box

# Load the video capture device
cap = cv2.VideoCapture(0)

# Load the template images
templates = []
for i in range(5):
    template = cv2.imread(f"template_{i}.jpg")
    templates.append(template)

# Initialize the previous bounding box
prev_box = None

while True:
    # Read a frame from the video capture device
    ret, frame = cap.read()

    # Detect the Coke can using the sliding window approach
    detected_box = detect_coke_can_sliding_window(frame, templates, prev_box)

    # Update the previous bounding box
    prev_box = detected_box

    # Display the output
    cv2.imshow("Coke Can Detector", frame)

    # Exit on key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture device
cap.release()
cv2.destroyAllWindows()