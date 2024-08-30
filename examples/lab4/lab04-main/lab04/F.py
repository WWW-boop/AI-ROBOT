import cv2
import numpy as np
import os
import matplotlib.pyplot as plt

# Define the path to the image file
img = cv2.imread("examples/04_camera/captured_image.jpg")

# Check if the image was loaded successfully
if img is not None:
    # Convert the image from BGR to HLS color space
    hls_image = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)

    # Extract the L-channel (lightness) from the HLS image
    l_channel = hls_image[:, :, 1]  # In HLS, L (lightness) is the second channel (index 1)

    # Rescale the L-channel to [-1, 1]
    l_channel_rescaled = (l_channel / 127.5) - 1

    # Display the rescaled L-channel image using OpenCV
    # Note: Since OpenCV does not directly support displaying images with negative values,
    # we will rescale the values back to the range [0, 255] for visualization purposes.
    l_channel_display = cv2.normalize(l_channel_rescaled, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
    l_channel_display = l_channel_display.astype(np.uint8)

    cv2.imshow("Rescaled L-Channel Image", l_channel_display)
    cv2.waitKey(0)  # Wait for a key press to close the window
    cv2.destroyAllWindows()

    # Print the original and rescaled L-channel image for verification
    print("Original L-channel range: [0, 255]")
    print("Rescaled L-channel range: [-1, 1]")
    print("Example rescaled L-channel pixel values:")
    print(l_channel_rescaled)

    # Plot the histogram of the rescaled L-channel image
    plt.hist(l_channel_rescaled.ravel(), bins=256, range=(-1, 1), fc='k', ec='k')
    plt.title('Histogram of Rescaled L-Channel Image')
    plt.xlabel('Pixel Value')
    plt.ylabel('Frequency')
    plt.show()
else:
    print("Failed to load image. Check if the file is a valid image format.")