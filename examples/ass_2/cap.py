import cv2
import time
import numpy as np
from robomaster import robot
from robomaster import camera
import matplotlib.pyplot as plt

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
        cv2.imwrite("bitchass23.jpg", img)
        print("Image captured and saved as 'captured_image.jpg'.")
    else:
        print("Failed to capture image.")

    # Stop the video stream
    ep_camera.stop_video_stream()

    # Close the robot connection
    ep_robot.close()