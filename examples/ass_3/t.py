import robomaster
from robomaster import robot
import time
import matplotlib.pyplot as plt
import csv
from datetime import datetime


ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")

def move_forward():
    print("move_forward called")
    for i in range(2):
        ep_robot.chassis.move(x=0.3, y=0, z=0, xy_speed=10).wait_for_completed()
        ep_robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
