import numpy as np
import cv2

img = cv2.imread("examples/04_camera/captured_image.jpg")
height, width, _ = img.shape

# Generate random noise with normal distribution (mean=0, variance=1)
noise_image = np.random.normal(0, 1, (height, width))

# Normalize the noise to fit the range [0, 255]
normalized_noise = cv2.normalize(noise_image, None, 0, 255, cv2.NORM_MINMAX)

# Convert to 8-bit unsigned integer
noise_image_uint8 = normalized_noise.astype(np.uint8)

# Save the noise image to a file
cv2.imwrite("noise_image.jpg", noise_image_uint8)

# Display the noise image
cv2.imshow('Random Noise Image', noise_image_uint8)
cv2.waitKey(0)
cv2.destroyAllWindows()