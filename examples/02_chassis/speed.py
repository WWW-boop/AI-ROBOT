import time
from robomaster import robot

# Initialize variables to store previous position and time
prev_x, prev_y, prev_z = None, None, None
prev_time = None

def sub_position_handler(position_info):
    global prev_x, prev_y, prev_z, prev_time

    # Get current position and time
    x, y, z = position_info
    current_time = time.time()

    # If previous position is not None, calculate the speed
    if prev_x is not None and prev_time is not None:
        # Calculate distance traveled
        distance = ((x - prev_x) ** 2 + (y - prev_y) ** 2 + (z - prev_z) ** 2) ** 0.5
        
        # Calculate time interval
        time_interval = current_time - prev_time
        
        # Calculate speed (distance / time)
        speed = distance / time_interval
        
        print(f"Chassis position: x:{x}, y:{y}, z:{z}")
        print(f"Speed: {speed:.2f} units/second")

    # Update previous position and time
    prev_x, prev_y, prev_z = x, y, z
    prev_time = current_time

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis

    # Subscribe to chassis position information
    ep_chassis.sub_position(freq=10, callback=sub_position_handler)
    
    # Move chassis
    ep_chassis.move(x=0.6, y=0, z=0).wait_for_completed()
    
    # Unsubscribe from position information
    ep_chassis.unsub_position()

    ep_robot.close()
