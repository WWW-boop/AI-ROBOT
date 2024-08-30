import csv
import matplotlib.pyplot as plt
from robomaster import robot

# Lists to store position data
x_data = []
y_data = []

# Callback function to handle position data
def sub_position_handler(position_info):
    x, y, z = position_info
    print("Chassis position: x:{0}, y:{1}, z:{2}".format(x, y, z))
    writer.writerow([x, y, z])
    x_data.append(x)
    y_data.append(y)

if __name__ == '__main__':
    # Initialize the robot
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    # Open a CSV file to write the position data
    with open('position_data.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["x", "y", "z"])

        ep_chassis = ep_robot.chassis

        # Set parameters for movement
        repetitions = 6
        see = 0
        count = 0

        # Subscribe to position data with a callback
        ep_chassis.sub_position(freq=50, callback=sub_position_handler)

        # Movement loop
        while count < 5:
            while see < 4:
                # Move straight
                for i in range(repetitions):
                    ep_chassis.move(x=0.1, y=0, z=0, xy_speed=0.7).wait_for_completed()
                    position = ep_chassis.get_position()
                    if position.y != 0:
                        ep_chassis.move(x=0, y=-position.y, z=0).wait_for_completed()
                
                # Turn
                for i in range(repetitions):
                    ep_chassis.move(x=0, y=0, z=-15, xy_speed=45).wait_for_completed()
                
                see += 1
            count += 1

        # Unsubscribe from position updates
        ep_chassis.unsub_position()

    # Close the robot connection
    ep_robot.close()

    # Plot the position data
    plt.figure(figsize=(10, 6))
    plt.plot(x_data, y_data, marker='o')
    plt.title("Robot Chassis Position")
    plt.xlabel("X position")
    plt.ylabel("Y position")
    plt.grid(True)
    plt.axis('equal')
    plt.show()
