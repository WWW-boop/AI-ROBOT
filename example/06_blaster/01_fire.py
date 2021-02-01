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


import time
from robomaster import robot
from robomaster import blaster


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta")

    ep_blaster = ep_robot.blaster

    # 發射1顆水彈
    ep_blaster.fire(times=1)
    time.sleep(2)

    # 發射3顆水彈
    ep_blaster.fire(fire_type=blaster.WATER_FIRE, times=3)
    time.sleep(2)

    # 發射1顆紅外子彈
    ep_blaster.fire(fire_type=blaster.INFRARED_FIRE)
    time.sleep(2)

    # 發射3顆紅外子彈
    ep_blaster.fire(fire_type=blaster.INFRARED_FIRE, times=3)
    time.sleep(2)

    ep_robot.close()
