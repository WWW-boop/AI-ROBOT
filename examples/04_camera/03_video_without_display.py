import cv2
from robomaster import robot
import time
import matplotlib.pyplot as plt

# Number of runs to collect execution times
num_runs = 5
execution_times = []

for _ in range(num_runs):
    start_time = time.time()

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_camera = ep_robot.camera

    ep_camera.start_video_stream(display=False)
    for i in range(0, 200):
        img = ep_camera.read_cv2_image()
        cv2.imshow("Robot", img)
        cv2.waitKey(1)
    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()

    ep_robot.close()

    end_time = time.time()
    execution_times.append(end_time - start_time)

# Plotting the execution times
plt.figure(figsize=(6, 6))
plt.plot(range(1, num_runs + 1), execution_times, marker='o', linestyle='-', color='b')
plt.title('Execution Time of Robot WIFI / TCP')
plt.xlabel('Run Number')
plt.ylabel('Execution Time (seconds)')
plt.grid(True)

# Save the plot as an image file
plt.savefig('execution_times_plot_w_T.png')

# Optionally display the plot
plt.show()
