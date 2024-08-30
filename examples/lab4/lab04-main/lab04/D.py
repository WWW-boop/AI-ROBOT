import cv2
import numpy as np
import matplotlib.pyplot as plt

# Define the path to the image file
img = cv2.imread("examples/04_camera/captured_image.jpg")

# Check if the file exists
    # Load the previously saved image

    # Check if the image was loaded successfully
if img is not None:
        # Convert the image from BGR to HLS color space
    hls_image = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)

        # Extract the L-channel (lightness) from the HLS image
    l_channel = hls_image[:, :, 1]  # In HLS, L (lightness) is the second channel (index 1)

        # Calculate the histogram of the L-channel
    hist = cv2.calcHist([l_channel], [0], None, [255], [0, 255])

        # Plot the histogram using Matplotlib
    plt.figure()
    plt.title("Histogram of L-Channel")
    plt.xlabel("Lightness value")
    plt.ylabel("Frequency")
    plt.plot(hist, color='black')
    plt.xlim([0, 255])
    plt.show()

else:
    print("Failed to load image. Check if the file is a valid image format.")

