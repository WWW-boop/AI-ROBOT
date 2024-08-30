import time
from robomaster import robot
import matplotlib.pyplot as plt

speed_data_x = []
speed_data_y = []
speed_data_z = []
timestamps = []

def speed_callback(speed_info):
    current_time = time.time()
    timestamps.append(current_time)
    speed_data_x.append(speed_info['x'])
    speed_data_y.append(speed_info['y'])
    speed_data_z.append(speed_info['z'])
    print(f"Speed data: x={speed_info['x']}, y={speed_info['y']}, z={speed_info['z']}")

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis

    x_val = 0.7

    # Register the callback function
    ep_chassis.sub_speed(freq=5, callback=speed_callback)  # subscribe to speed data at 5Hz

    ep_chassis.drive_speed(x=x_val, y=0, z=0, timeout=5)

    # Sleep to allow time for speed data to be collected
    time.sleep(5)

    ep_robot.close()

    # Plot the speed data
    # Adjust timestamps to be relative to the start time for better readability
    start_time = timestamps[0]
    relative_times = [t - start_time for t in timestamps]

    plt.figure(figsize=(10, 6))
    plt.plot(relative_times, speed_data_x, label='Speed X')
    plt.plot(relative_times, speed_data_y, label='Speed Y')
    plt.plot(relative_times, speed_data_z, label='Speed Z')
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (m/s)')
    plt.title('Robot Speed Data')
    plt.legend()
    plt.grid(True)
    plt.show()
