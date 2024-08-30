


from robomaster import robot
import pandas as pd
import time

def sub_position_handler(position_info):
    x, y, z = position_info
    print("chassis position: x:{0}, y:{1}, z:{2}".format(x, y, z))
    dict = {x.y.z}
    df = pd.DataFrame(dict)
    print (df)
    df.to_csv('file.csv')


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis
    ep_chassis.sub_position(freq=10, callback=sub_position_handler)
    
    x_val = 1
    
    
    
    ep_chassis.move(x=0, y=0, z=180, xy_speed=1).wait_for_completed()
        #time.sleep(1)
    #ep_chassis.move(x=-x_val, y=0, z=0, xy_speed=1).wait_for_completed()
        #time.sleep(1)
    

    

    ep_chassis.unsub_position()

    ep_robot.close()
