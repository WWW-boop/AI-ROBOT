import robomaster
from robomaster import robot
import time

def sub_position_handler(position_info):
    x = position_info
    print("chassis position: x:{0}".format(x))

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis

    # Subscribe to position information with a frequency of 10Hz
    ep_chassis.sub_position(freq=10, callback=sub_position_handler)

    # Record the start time
    start_time = time.time()

    # Move the chassis
    ep_chassis.move(x=0.7).wait_for_completed()

    # Record the end time
    end_time = time.time()

    # Unsubscribe from position information
    ep_chassis.unsub_position()

    # Calculate and print the time duration
    duration = end_time - start_time
    print("Movement duration: {:.2f} seconds".format(duration))

    # Close the robot connection
    ep_robot.close()