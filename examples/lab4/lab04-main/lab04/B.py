# -*-coding:utf-8-*-
# Copyright (c) 2020 DJI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import cv2

# Load the previously saved image
img = cv2.imread("examples/04_camera/captured_image.jpg")

# Check if the image was loaded successfully
if img is not None:
    # Print the size of the image
    height, width, channels = img.shape
    print(f"Image size: Width = {width} pixels, Height = {height} pixels, Channels = {channels}")

    # Show the original image
    cv2.imshow("Original Image", img)
    cv2.waitKey(0)  # Wait for a key press to close the window
    cv2.destroyAllWindows()
else:
    print("Failed to load image.")
