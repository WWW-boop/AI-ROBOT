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
import time
from robomaster import robot
from robomaster import camera


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_camera = ep_robot.camera

    # Start video stream with 720P resolution
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    
    # Allow some time for the stream to start
    time.sleep(2)

    # Capture an image from the video stream
    img = ep_camera.read_cv2_image(strategy="newest")

    # Save the captured image to a file
    if img is not None:
        cv2.imwrite("coke-4block.jpg", img)
        print("Image captured and saved as 'captured_image.jpg'.")
    else:
        print("Failed to capture image.")

    # Stop the video stream
    ep_camera.stop_video_stream()

    # Close the robot connection
    ep_robot.close()
