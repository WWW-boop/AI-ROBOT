import cv2
import numpy as np

# Load the original image
img = cv2.imread("examples/04_camera/captured_image.jpg")

# Check if the image was loaded successfully
if img is not None:
    # Convert the image from BGR to HLS color space
    hls_image = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)

    # Extract the L-channel (lightness) from the HLS image
    l_channel = hls_image[:, :, 1]  # In HLS, L (lightness) is the second channel (index 1)

    # Rescale the L-channel to [-1, 1]
    l_channel_rescaled = (l_channel / 127.5) - 1

    # Load the noise image
    noise_image = cv2.imread("examples/04_camera/noise_image.jpg", cv2.IMREAD_GRAYSCALE)

    # Ensure both images have the same dimensions
    if noise_image.shape != l_channel_rescaled.shape:
        noise_image = cv2.resize(noise_image, (l_channel_rescaled.shape[1], l_channel_rescaled.shape[0]))

    # Combine the noise image with the rescaled L-channel image
    combined_image = l_channel_rescaled + (noise_image / 127.5 - 1)

    # Normalize the combined image to fit the range [0, 255] for visualization
    combined_image_display = cv2.normalize(combined_image, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
    combined_image_display = combined_image_display.astype(np.uint8)

    # Save and display the combined image
    cv2.imwrite("combined_image.jpg", combined_image_display)
    cv2.imshow("Combined Image", combined_image_display)
    cv2.waitKey(0)  # Wait for a key press to close the window
    cv2.destroyAllWindows()

    # Print the original and combined L-channel image for verification
    print("Original L-channel range: [0, 255]")
    print("Rescaled L-channel range: [-1, 1]")
    print("Example combined L-channel pixel values:")
    print(combined_image)
else:
    print("Failed to load image. Check if the file is a valid image format.")