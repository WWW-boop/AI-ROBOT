import time
import matplotlib.pyplot as plt
from robomaster import robot

axis_x = []
axis_y = []
time_all = []
target = []

global expect
expect = 2

def sub_position_handler(position_info):
    x, y, z = position_info
    print(x)
    axis_x.append(x)
    axis_y.append(y)
    end_time = time.time() - start_time
    final_endtime = '{:.05f}'.format(end_time)
    time_all.append(final_endtime)
    target.append(expect)

def toggle_target(expect):
    
    return 2 - expect


if __name__ == '__main__':
    
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_chassis.sub_position(freq=50, callback=sub_position_handler)
    p = 200
    kd = 5
    start_time = time.time()
    pre_time = 0
    pre_err = 0
    times = 0

    while times < 2:
        if axis_x and time_all:
            now_x = axis_x[-1]
            err = expect - now_x
            now_time = float(time_all[-1])
            diff_time = now_time - pre_time
            diff_err = err - pre_err
            if diff_time > 0:
                speed = err*p+(kd * (diff_err / diff_time))
                print(expect)
                err_abs = abs(err)
                pre_time = now_time
                pre_err = err
                if err_abs < 0.001:
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
    ep_robot.close()
    
    
    plt.figure(figsize=(6, 6))
    plt.plot(time_all, target, label='Target')
    plt.plot(time_all, axis_x, label='X Position')
    plt.title('Expected and actual')
    plt.ylabel('position')
    plt.xlabel('Time')
    plt.legend()
    plt.tight_layout()
    plt.show()

