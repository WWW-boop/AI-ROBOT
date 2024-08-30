import robomaster
from robomaster import robot
import numpy as np
import time
import matplotlib.pyplot as plt

tof_distance = None
adc_left_filter = None
adc_right_filter = None
MAX_SPEED = 2.5
WALL_DISTANCE_THRESHOLD = 20.0
yaw = None
count = 0
stop_count = 0

status_tof = False
position_robot = []
tof_value = []
adc_values_left = []
adc_values_right = []

# adc_cm_left = 0
# adc_cm_right = 0
# status_ss_right = None 
# status_ss_left = None

def tof_filter(new_value, previous_filtered):
    alpha = 0.1
    if previous_filtered is None:
        previous_filtered = 0
    return alpha * previous_filtered  +  (1 - alpha) * new_value

def adc_filter(new_value, previous_filtered):
    alpha = 0.1
    if previous_filtered is None:
        previous_filtered = 0
    return alpha * previous_filtered  +  (1 - alpha) * new_value


def sub_position_handler(position_info):
    x, y, z = position_info
    position_robot.append((x, y, z))


def sub_attitude_info_handler(attitude_info):
    global yaw
    yaw, pitch, roll = attitude_info


def tof_data_handler(sub_info):

    global tof_distance, status_tof, adc_cm_left, adc_cm_right, adc_left_filter, adc_right_filter, adc_1_left, adc_1_right, status_ss_left, status_ss_right

    if tof_distance is not None:
        tof_distance = tof_filter(sub_info[0], tof_distance)
        tof_value.append(tof_distance)
    else:
        tof_distance = sub_info[0]

    if tof_distance < 160:
        status_tof = True
    elif tof_distance >160:
        status_tof = False

    # wait add filter signal for adc
    adc_1_left = ep_sensor_adaptor.get_adc(id=1, port=1)
    adc_1_right = ep_sensor_adaptor.get_adc(id=2, port=2)

    print('distance from left wall : {}'.format(adc_1_left))

    if adc_left_filter is not None:
        adc_left_filter = adc_filter(adc_1_left, adc_left_filter)
    else:
        adc_left_filter = adc_1_left

    print('filtered_adc_left :',adc_left_filter)

    print('distance from right wall : {}'.format(adc_1_right))

    if adc_right_filter is not None:
        adc_right_filter = adc_filter(adc_1_right, adc_right_filter)
    else:
        adc_right_filter = adc_1_right

    print('filtered_adc_right :',adc_right_filter)

    adc_2_left = (adc_left_filter * 3) / 1023  # process to cm unit
    adc_2_right = (adc_right_filter * 3) / 1023  # process to cm unit

    if 2.2 <= adc_2_left < 3.2:
        adc_cm_left = (adc_2_left - 4.30764) / -0.3846
    elif 1.4 <= adc_2_left < 2.2:
        adc_cm_left = (adc_2_left - 3.2) / -0.2
    elif 0.8 <= adc_2_left < 1.4:
        adc_cm_left = (adc_2_left - 1.87) / -0.067
    elif 0.4 <= adc_2_left < 0.8:
        adc_cm_left = (adc_2_left - 1.344) / -0.034
    else:
        if adc_2_left >= 3.2:
            adc_cm_left = (adc_2_left - 4.30764) / -0.3846
        elif adc_2_left < 0.4:
            adc_cm_left = (adc_2_left - 1.344) / -0.034

    if 2.2 <= adc_2_right < 3.2:
        adc_cm_right = (adc_2_right - 4.30764) / -0.3846
    elif 1.4 <= adc_2_right < 2.2:
        adc_cm_right = (adc_2_right - 3.2) / -0.2
    elif 0.8 <= adc_2_right < 1.4:
        adc_cm_right = (adc_2_right - 1.87) / -0.067
    elif 0.4 <= adc_2_right < 0.8:
        adc_cm_right = (adc_2_right - 1.344) / -0.034
    else:
        if adc_2_right >= 3.2:
            adc_cm_right = (adc_2_right - 4.30764) / -0.3846
        elif adc_2_right < 0.4:
            adc_cm_right = (adc_2_right - 1.344) / -0.034

    adc_values_left.append(adc_cm_left)
    adc_values_right.append(adc_cm_right)

    if 23 >= adc_cm_left >= 7:
        status_ss_left = True
    else:
        status_ss_left = False

    if 23 >= adc_cm_right >= 7:
        status_ss_right = True
    else:
        status_ss_right = False

    time.sleep(0.5)

    # print("Distance from front wall : {} mm".format(tof_distance))
    # print("See front wall :", status_tof)
    # print("See right wall :", status_ss_right)


def forward(drive_wheel_speed):
    ep_chassis.drive_wheels(
        w1=drive_wheel_speed,
        w2=drive_wheel_speed,
        w3=drive_wheel_speed,
        w4=drive_wheel_speed,
    )


def backward():
    ep_chassis.move(x=0, y=0, z=-180, z_speed=45).wait_for_completed()


def move(move_x):
    ep_chassis.move(x=move_x, y=0, z=0, xy_speed=MAX_SPEED).wait_for_completed()


def turn_right():
    ep_chassis.move(x=0, y=0, z=-90, z_speed=90).wait_for_completed()


def turn_left():
    ep_chassis.move(x=0, y=0, z=90, z_speed=90).wait_for_completed()


def stop():
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.75)
    time.sleep(1)


def check_stop():
    val = abs(position_robot[-1][0] - position_robot[0][0])
    print(f"Position difference: {val}")
    if val < 0.05:
        stop()
        print("Robot has returned to the start position. Stopping.")
        return True
    else:
        print("Robot hasn't returned to the start position yet.")
        return False


def plot_robot_data(position_robot, tof_value, adc_values_left, adc_values_right):
    # Extract x, y, z coordinates from position_robot
    x_positions = [pos[0] for pos in position_robot]
    y_positions = [pos[1] for pos in position_robot]
    z_positions = [pos[2] for pos in position_robot]

    # Create time array (assuming constant time intervals)
    time_array = np.arange(len(position_robot))

    # Plot X Position
    plt.subplot(2, 2, 1)
    plt.plot(time_array, x_positions)
    plt.ylabel("X Position")
    plt.title("X Position")

    # Plot Y Position
    plt.subplot(2, 2, 2)
    plt.plot(time_array, y_positions)
    plt.ylabel("Y Position")
    plt.title("Y Position")

    # Plot ToF Values
    plt.subplot(2, 2, 3)
    plt.plot(time_array[: len(tof_value)], tof_value)
    plt.ylabel("ToF Distance")
    plt.title("ToF Position")

    # Plot ADC Values
    plt.subplot(2, 2, 4)
    plt.plot(time_array[: len(adc_values_left)], adc_values_left, "c-", label="Left")
    plt.plot(time_array[: len(adc_values_right)], adc_values_right, "y-", label="Right")
    plt.ylabel("ADC Values")
    plt.title("ADC Values")

    # Adjust layout and display the plot
    plt.grid()
    plt.suptitle("Robomaster's signals")
    plt.legend()
    plt.show()


print("**************************")

if __name__ == "__main__":
    ep_robot = robot.Robot()
    print("Initializing robot...")
    ep_robot.initialize(conn_type="ap")
    time.sleep(1)

    # เริ่มต้นการทำงานของเซ็นเซอร์ต่าง ๆ
    ep_sensor = ep_robot.sensor
    ep_chassis = ep_robot.chassis
    ep_gimbal = ep_robot.gimbal
    ep_sensor_adaptor = ep_robot.sensor_adaptor

    ep_sensor.sub_distance(freq=10, callback=tof_data_handler)
    ep_chassis.sub_attitude(freq=10, callback=sub_attitude_info_handler)
    ep_chassis.sub_position(freq=10, callback=sub_position_handler)

    ep_gimbal.recenter().wait_for_completed()

    try:
        while True:
            if tof_distance is None or adc_cm_left is None or adc_cm_right is None:
                print("Waiting for sensor data...")
                time.sleep(1)
                continue

            gap = abs(WALL_DISTANCE_THRESHOLD - adc_cm_right)
            walk_y = gap / 100

            while status_tof == False and status_ss_right == True and stop_count == 0:
                forward(50)
                if count == 1 and check_stop():
                    stop()
                    stop_count += 1
                    break

            if stop_count > 0:
                break

            # dead end
            if status_tof == True :
                if tof_distance < 160:
                    ep_chassis.move(
                        x=(tof_distance - 160) / 1000, y=0, z=0, xy_speed=MAX_SPEED
                    ).wait_for_completed()
                stop()
                time.sleep(0.5)
                backward()
                time.sleep(0.9)
                move(0.35)
                time.sleep(0.5)
                count += 1

            # move left / right
            elif status_tof == False and status_ss_right == False:
                stop()
                if adc_cm_right < WALL_DISTANCE_THRESHOLD - 4:
                    ep_chassis.move(
                        x=0, y=-walk_y, z=0, xy_speed=MAX_SPEED * 1.3
                    ).wait_for_completed()
                elif adc_cm_right > WALL_DISTANCE_THRESHOLD + 4:
                    ep_chassis.move(
                        x=0, y=walk_y, z=0, xy_speed=MAX_SPEED * 1.3
                    ).wait_for_completed()

    except KeyboardInterrupt:
        print("Program stopped by user")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print("Cleaning up...")
        # print(tof_value)
        ep_sensor.unsub_distance()
        stop()
        time.sleep(0.01)
        ep_robot.close()
        plot_robot_data(position_robot, tof_value, adc_values_left, adc_values_right)
        print("Program ended.")