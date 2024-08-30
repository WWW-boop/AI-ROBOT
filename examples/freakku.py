import robomaster
import time
import csv
from robomaster import robot
import pandas
import matplotlib.pyplot as plt

axis_x = []
axis_y = []
time_all = []
target = []


global expect
expect = 1

def sub_position_handler(position_info):
    x, y, z = position_info
    axis_x.append(x)
    axis_y.append(y)
    end_time = time.time()-start_time
    final_endtime= '{:.05f}'.format(end_time)
    time_all.append(final_endtime)
    target.append(expect)

def toggle_target(expect):
    return 1 - expect

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_chassis.sub_position(freq=50, callback=sub_position_handler)

    p = 50
    i = 5
    d = 1
    integral = 0
    previous_error = 0

    start_time = time.time()
    times = 0
    count = 2 # จำนวนครั้ง

    while times < count:
        if axis_x and time_all:
            now_x = axis_x[-1]
            now_time = float(time_all[-1])
            error = expect - now_x
            integral += error
            derivative = error - previous_error
            previous_error = error
            speed = (error * p) + (integral * i) + (derivative * d)
            err_abs = abs(error)
            
            if err_abs < 0.001:
                ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
                time.sleep(3)
                expect = toggle_target(expect)
                times += 1
                integral = 0  # Reset integral
            elif err_abs < 0.2:
                speed = (error * p + integral * i + derivative * d) * 2.5
            
            ep_chassis.drive_wheels(w1=speed, w2=speed, w3=speed, w4=speed)

    ep_robot.close()

    plt.figure(figsize=(6, 6))
    plt.plot(time_all, target, label='Expected')
    plt.plot(time_all, axis_x, label='Actual')
    plt.title('Expected and Actual Positions')
    plt.ylabel('Position')
    plt.xlabel('Time')
    plt.legend()
    plt.tight_layout()
    plt.show()
