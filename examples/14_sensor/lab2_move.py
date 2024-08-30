import csv
import time
from datetime import datetime
from robomaster import robot
x_data = []
y_data = []

# Create a CSV file and write the headers

with open('robot_data.csv', 'w', newline='') as csvfile:
   fieldnames = ['timestamp', 'x', 'y', 'z', 'tof1', 'yaw', 'pitch', 'roll', 'acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z', 'speed', 'angle', 'esc_timestamp', 'esc_state']
   writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
   writer.writeheader()

def sub_position_handler(position_info):
   x, y, z = position_info
   timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
   with open('robot_data.csv', 'a', newline='') as csvfile:
       writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
       writer.writerow({'timestamp': timestamp, 'x': x, 'y': y, 'z': z})
   print("chassis position: x:{0}, y:{1}, z:{2}".format(x, y, z))

def sub_data_handler(sub_info):
   distance = sub_info
   timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
   with open('robot_data.csv', 'a', newline='') as csvfile:
       writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
       writer.writerow({'timestamp': timestamp, 'tof1': distance[0]})
   print("tof1:{0}".format(distance[0]))

def sub_attitude_info_handler(attitude_info):
   yaw, pitch, roll = attitude_info
   timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
   with open('robot_data.csv', 'a', newline='') as csvfile:
       writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
       writer.writerow({'timestamp': timestamp, 'yaw': yaw, 'pitch': pitch, 'roll': roll})
   print("chassis attitude: yaw:{0}, pitch:{1}, roll:{2}".format(yaw, pitch, roll))

def sub_imu_info_handler(imu_info):
   acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = imu_info
   timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
   with open('robot_data.csv', 'a', newline='') as csvfile:
       writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
       writer.writerow({'timestamp': timestamp, 'acc_x': acc_x, 'acc_y': acc_y, 'acc_z': acc_z, 'gyro_x': gyro_x, 'gyro_y': gyro_y, 'gyro_z': gyro_z})
   print("chassis imu: acc_x:{0}, acc_y:{1}, acc_z:{2}, gyro_x:{3}, gyro_y:{4}, gyro_z:{5}".format(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z))

def sub_esc_info_handler(esc_info):
   speed, angle, esc_timestamp, state = esc_info
   timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
   with open('robot_data.csv', 'a', newline='') as csvfile:
       writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
       writer.writerow({'timestamp': timestamp, 'speed': speed, 'angle': angle, 'esc_timestamp': esc_timestamp, 'esc_state': state})
   print("chassis esc: speed:{0}, angle:{1}, timestamp:{2}, state:{3}".format(speed, angle, esc_timestamp, state))

if __name__ == '__main__':
   for i in range(0, 3):
       ep_robot = robot.Robot()
       ep_robot.initialize(conn_type="ap")
       ep_sensor = ep_robot.sensor
       ep_chassis = ep_robot.chassis
       x_val = 1
       y_val = 1
       ep_chassis.sub_position(freq=5, callback=sub_position_handler)
       ep_sensor.sub_distance(freq=5, callback=sub_data_handler)
       ep_chassis.sub_attitude(freq=5, callback=sub_attitude_info_handler)
       ep_chassis.sub_imu(freq=5, callback=sub_imu_info_handler)
       ep_chassis.sub_esc(freq=5, callback=sub_esc_info_handler)

       ep_chassis.move(x=x_val, y=0, z=0, xy_speed=14).wait_for_completed()
       ep_chassis.move(x=0, y=y_val, z=0, xy_speed=14).wait_for_completed()
       ep_chassis.move(x=-x_val, y=0, z=0, xy_speed=14).wait_for_completed()
       ep_chassis.move(x=0, y=-y_val, z=0, xy_speed=14).wait_for_completed()

       ep_chassis.unsub_imu()
       ep_sensor.unsub_distance()
       ep_chassis.unsub_position()
       ep_chassis.unsub_attitude()
       ep_chassis.unsub_esc()
       ep_robot.close()