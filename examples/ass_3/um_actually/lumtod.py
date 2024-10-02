import cv2
import numpy as np

def get_shape_name(approx):
    # Determine the shape based on the number of vertices
    num_vertices = len(approx)
    if num_vertices == 3:
        return "Triangle"
    elif num_vertices == 4:
        aspect_ratio = cv2.contourArea(approx) / (cv2.boundingRect(approx)[2] * cv2.boundingRect(approx)[3])
        if 0.95 <= aspect_ratio <= 1.05:
            return "Square"
        else:
            return "Rectangle"
    elif num_vertices == 5:
        return "Pentagon"
    elif num_vertices > 5:
        return "Circle"
    return "Unknown"

def bro_thinks_its_chicken(chicken_contour, img):
    chicken_contour_max = max(chicken_contour, key=cv2.contourArea)
    chicken_x_axis, chicken_y_axis, chicken_width, chicken_height = cv2.boundingRect(chicken_contour_max)
    
    # Approximate contour to get the shape
    epsilon = 0.02 * cv2.arcLength(chicken_contour_max, True)
    approx = cv2.approxPolyDP(chicken_contour_max, epsilon, True)
    shape_name = get_shape_name(approx)

    # Adjust bounding box size based on width
    if chicken_width > 30:
        weight_factor, height_factor, y_offset, x_offset, w_offset, h_offset = 0.5, 0.6, 10, 5, -10, -5
    elif chicken_width > 15:
        weight_factor, height_factor, y_offset, x_offset, w_offset, h_offset = 0.5, 0.6, 5, 2, 1, 1
    else:
        weight_factor, height_factor, y_offset, x_offset, w_offset, h_offset = 0.5, 0.6, 1, 1, 2, 2

    adjust_width = int(chicken_width * weight_factor) + w_offset
    adjust_height = int(chicken_height * height_factor) + h_offset
    new_chicken_x = chicken_x_axis - adjust_width // 2 + x_offset
    new_chicken_y = chicken_y_axis - adjust_height // 2 + y_offset
    new_chicken_width = chicken_width + adjust_width
    new_chicken_height = chicken_height + adjust_height

    # Draw the chicken bounding box and label
    cv2.rectangle(img, 
                  (new_chicken_x, new_chicken_y), 
                  (new_chicken_x + new_chicken_width, new_chicken_y + new_chicken_height), 
                  (0, 255, 0), 2)
    cv2.putText(img, f"Chicken: {shape_name}", 
                (new_chicken_x, new_chicken_y - 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

    cv2.imshow("Chicken Detection", img)

def bro_thinks_its_bottle(bottle_contour, img):
    bottle_contour_max = max(bottle_contour, key=cv2.contourArea)
    bottle_x, bottle_y, bottle_width, bottle_height = cv2.boundingRect(bottle_contour_max)

    # Approximate contour to get the shape
    epsilon = 0.02 * cv2.arcLength(bottle_contour_max, True)
    approx = cv2.approxPolyDP(bottle_contour_max, epsilon, True)
    shape_name = get_shape_name(approx)

    # Adjust the bounding box size
    if bottle_width > 70:
        width_factor, height_factor, y_offset, x_offset, w_offset, h_offset = 1.5, 3, 0, 0, 0, 50
    elif bottle_width > 35:
        width_factor, height_factor, y_offset, x_offset, w_offset, h_offset = 1.5, 3, 0, 0, 0, 50
    else:
        width_factor, height_factor, y_offset, x_offset, w_offset, h_offset = 1.5, 3, 0, 0, 0, 50

    adjust_width = int(bottle_width * width_factor) + w_offset
    adjust_height = int(bottle_height * height_factor) + h_offset
    new_bottle_x = bottle_x - adjust_width // 2 + x_offset
    new_bottle_y = bottle_y + y_offset
    new_bottle_width = bottle_width + adjust_width
    new_bottle_height = bottle_height + adjust_height

    # Draw the updated bounding box and label
    cv2.rectangle(img, 
                  (new_bottle_x, new_bottle_y), 
                  (new_bottle_x + new_bottle_width, new_bottle_y + new_bottle_height), 
                  (0, 255, 0), 1)
    cv2.putText(img, f"Bottle: {shape_name}", 
                (new_bottle_x, new_bottle_y - 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

    cv2.imshow("Bottle Detection", img)

def bro_thinks_its_circle(circles, img):
    # Check if any circles are detected
    if circles is not None:
        # Convert the (x, y, radius) values to integers
        circles = np.round(circles[0, :]).astype("int")
        
        # Draw detected circles
        for circle in circles:
            center = (circle[0], circle[1])
            radius = circle[2]
            # Draw the circle in the output image
            cv2.circle(img, center, radius, (255, 0, 0), 1)  # Circle outline
            cv2.putText(img, "Circle", (center[0] - 20, center[1] - radius - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)

        cv2.imshow("Circle Detection", img)
    else:
        print("No circles detected.")

if __name__ == "__main__":
    image_path = r"C:\Users\lataeq\AI-ROBOT\examples\ass_3\lataeq\pics\agent.jpg"  # Replace with the path to your image
    img = cv2.imread(image_path)

    if img is None:
        print(f"Failed to load image from {image_path}")
        exit()

    # Step 1: Show the original image
    cv2.imshow("Original Image", img)

    # Preprocess image
    background = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    background = cv2.GaussianBlur(background, (21, 21), 0)
    cv2.imshow("Blurred Background", background)

    # Step 2: Create the blurred grayscale version
    gray_blurred = cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (21, 21), 0)
    cv2.imshow("Gray Blurred", gray_blurred)

    # Step 3: Compute the absolute difference
    diff = cv2.absdiff(background, gray_blurred)
    cv2.imshow("Difference", diff)

    # Step 4: Thresholding
    _, img_threshold = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)
    cv2.imshow("Threshold", img_threshold)

    # Step 5: Dilation
    img_threshold = cv2.dilate(img_threshold, None, iterations=2)
    cv2.imshow("Dilated Threshold", img_threshold)

    # Step 6: Convert to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    cv2.imshow("HSV Image", hsv)

    # Step 7: Create masks
    lower_hue_chicken = np.array([33, 150, 100])
    upper_hue_chicken = np.array([36, 255, 255])
    lower_hue_bottle = np.array([35, 132, 0])
    upper_hue_bottle = np.array([255, 255, 255])

    chicken_mask = cv2.inRange(hsv, lower_hue_chicken, upper_hue_chicken)
    bottle_mask = cv2.inRange(hsv, lower_hue_bottle, upper_hue_bottle)
    
    # Step 8: Show masks
    cv2.imshow("Chicken Mask", chicken_mask)
    cv2.imshow("Bottle Mask", bottle_mask)

    # Step 9: Find contours
    chicken_contour, _ = cv2.findContours(chicken_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    bottle_contour, _ = cv2.findContours(bottle_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Step 10: Process detected contours
    if chicken_contour: 
        bro_thinks_its_chicken(chicken_contour, img)
    if bottle_contour:
        bro_thinks_its_bottle(bottle_contour, img)

    # Step 11: Detect circles using Hough Transform
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 5)
    circles = cv2.HoughCircles(gray, 
                                 cv2.HOUGH_GRADIENT, 
                                 dp=1, 
                                 minDist=20,
                                 param1=50, 
                                 param2=30, 
                                 minRadius=1, 
                                 maxRadius=30)

    bro_thinks_its_circle(circles, img)

    # Final image after processing
    cv2.imshow("Final Output", img)

    cv2.waitKey(0)
    cv2.destroyAllWindows()
