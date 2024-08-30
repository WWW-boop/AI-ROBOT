import cv2
import numpy as np
import matplotlib.pyplot as plt

# Define the path to the image file
img = cv2.imread("examples/04_camera/captured_image.jpg")

# Check if the image was loaded successfully
if img is not None:
    # Convert the image from BGR to HLS color space
    hls_image = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)

    # Extract the L-channel (lightness) from the HLS image
    l_channel = hls_image[:, :, 1]  # In HLS, L (lightness) is the second channel (index 1)
    # Normalize the L-channel to the range [0, 255]
    l_channel_normalized = cv2.normalize(l_channel, None, 0, 255, cv2.NORM_MINMAX)

    # Replace the L-channel in the HLS image with the normalized L-channel
    hls_image[:, :, 1] = l_channel_normalized

    # Convert the HLS image back to the BGR color space
    bgr_image = cv2.cvtColor(hls_image, cv2.COLOR_HLS2BGR)

    # Display the resulting image
    cv2.imshow("Result Image", bgr_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

else:
    print("Failed to load image. Check if the file is a valid image format.")