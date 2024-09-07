import cv2
import time
from robomaster import robot

# Initialize the robot
ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")

# Initialize the video stream from the camera
ep_camera = ep_robot.camera

# Set the camera resolution to 720p (1280x720)
ep_camera.start_video_stream(display=False, resolution="720p")

# Set up the video writer to record the stream to an mp4 file
video_writer = cv2.VideoWriter('output.mp4', 
                               cv2.VideoWriter_fourcc(*'mp4v'), 
                               30,  # 30 FPS
                               (1280, 720))

# Record for a certain amount of time (e.g., 10 seconds)
start_time = time.time()
duration = 20  # seconds

while time.time() - start_time < duration:
    frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
    if frame is not None:
        video_writer.write(frame)
    else:
        print("Failed to grab frame")

# Release resources
video_writer.release()
ep_camera.stop_video_stream()
ep_robot.close()

print("Video recording completed!")