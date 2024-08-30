import csv
import matplotlib.pyplot as plt
from robomaster import robot

x_data = []
y_data = []

def sub_position_handler(position_info):
    x, y, z = position_info
    print("chassis position: x:{0}, y:{1}, z:{2}".format(x, y, z))
    writer.writerow([x, y, z])
    x_data.append(x)
    y_data.append(y)

if __name__ == '__main__':
    
    with open('position_data.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["x", "y", "z"])
        
        for i in range(0,3): 
            ep_robot = robot.Robot()
            ep_robot.initialize(conn_type="ap")

            ep_chassis = ep_robot.chassis

            x_val = 1
            y_val = 1

            ep_chassis.sub_position(freq=5, callback=sub_position_handler)
            
            ep_chassis.move(x=x_val, y=0, z=0, xy_speed=14).wait_for_completed()
            ep_chassis.move(x=0, y=y_val, z=0, xy_speed=14).wait_for_completed()
            ep_chassis.move(x=-x_val, y=0, z=0, xy_speed=14).wait_for_completed()
            ep_chassis.move(x=0, y=-y_val, z=0, xy_speed=14).wait_for_completed()
                
            ep_chassis.unsub_position()

            ep_robot.close()

    plt.figure(figsize=(6, 6))
    plt.plot(x_data, y_data, marker='_')
    plt.title("Robot Chassis Position")
    plt.xlabel("X position")
    plt.ylabel("Y position")
    plt.grid(True)
    plt.axis('equal')
    plt.show()

