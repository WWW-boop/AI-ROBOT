# -*-coding:utf-8-*-
# Copyright (c) 2020 DJI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import robomaster
import time
from robomaster import robot


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis

    # 指定麦轮速度
    speed = 50
    slp = 3

    ep_chassis.drive_wheels(w1=speed, w2=speed, w3=speed, w4=speed)
    time.sleep(slp)

    ep_chassis.drive_wheels(w1=-speed, w2=speed, w3=-speed, w4=speed)
    time.sleep(slp)

    ep_chassis.drive_wheels(w1=-speed, w2=-speed, w3=-speed, w4=-speed)
    time.sleep(slp)

    ep_chassis.drive_wheels(w1=speed, w2=-speed, w3=speed, w4=-speed)
    time.sleep(slp)

    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

    ep_robot.close()

