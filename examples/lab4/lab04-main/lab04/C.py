import cv2

# Load the previously saved image
img = cv2.imread("examples/04_camera/captured_image.jpg")

# Check if the image was loaded successfully
if img is not None:
        # Convert the image from BGR to HLS color space
        hls_image = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)

        # Extract the L-channel (lightness) from the HLS image
        l_channel = hls_image[:, :, 1]  # In HLS, L (lightness) is the second channel (index 1)

        # Display the L-channel image
        cv2.imwrite("l_chanel_image.jpg", l_channel)
        cv2.imshow("L-Channel Image", l_channel)
        cv2.waitKey(0)  # Wait for a key press to close the window
        cv2.destroyAllWindows()
else:
    print("Failed to load image. Check if the file is a valid image format.")