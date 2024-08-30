import cv2
import numpy as np

# Load images
noise_image = cv2.imread('noise_image.jpg', cv2.IMREAD_GRAYSCALE)
l_channel_image = cv2.imread('l_channel_image.jpg', cv2.IMREAD_GRAYSCALE)

# Rescale L-channel image to match the dimensions of the noise image
l_channel_image_rescaled = cv2.resize(l_channel_image, (noise_image.shape[1], noise_image.shape[0]))

# Combine images (example: averaging the pixel values)
combined_image = cv2.addWeighted(noise_image, 0.5, l_channel_image_rescaled, 0.5, 0)

# Display the result
cv2.imshow('Combined Image', combined_image)
cv2.waitKey(0)
cv2.destroyAllWindows()