import time
import matplotlib.pyplot as plt
from robomaster import robot

# Initialize lists for storing data
axis_x = []
axis_y = []
time_all = []
target = []

# Initial target value
expect = 1

def sub_position_handler(position_info):
    x, y, z = position_info
    print(x)
    axis_x.append(x)
    axis_y.append(y)
    end_time = time.time() - start_time
    final_endtime = '{:.05f}'.format(end_time)
    time_all.append(final_endtime)
    target.append(expect)

# Function to toggle target value
def toggle_target(current_target):
    return 0 if current_target == 1 else 1


def main():
    global expect, start_time

    # Initialize robot and chassis
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_chassis.sub_position(freq=50, callback=sub_position_handler)

    p = 150.00
    kd = -50
    start_time = time.time()
    pre_time = 0
    pre_err = 0
    times = 0

    # Start playing the audio
    #ep_robot.play_audio(filename="C:/Users/ASUS/Desktop/Robot3/michael.wav")

    # Loop back and forth 5 times
    # Loop back and forth 5 times
    while times < 2:
        if axis_x and time_all:
            # print(expect)
            now_x = axis_x[-1]
            err = expect - now_x
            now_time = float(time_all[-1])
            diff_time = now_time - pre_time
            diff_err = err - pre_err
            if diff_time > 0:
                new_kd = kd * (diff_err / diff_time)
                speed = err * p  + (new_kd)
                print(expect)
                err_abs = abs(err)
                pre_time = now_time
                pre_err = err
                if err_abs < 0.001:
                    # print('Toggleeeeee')
                    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0, timeout = 0.2)
                    time.sleep(1)
                    expect = toggle_target(expect)
                    times += 1
            if speed < 12.6 and speed > 0:
                speed = 12.6
            elif speed > -12.6 and speed < 0:
                speed = -12.6
            ep_chassis.drive_wheels(w1=speed, w2=speed, w3=speed, w4=speed)
    ep_chassis.unsub_position()
# Stop the robot and close the connection
    ep_robot.close()
    plot_results()

def plot_results():
    plt.figure(figsize=(6, 6))
    plt.plot(time_all, target, label='Target')
    plt.plot(time_all, axis_x, label='X Position')
    plt.title('Robot MAP')
    plt.ylabel('X position')
    plt.xlabel('Time')
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()