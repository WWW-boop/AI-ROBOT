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
from robomaster import robot
import time

sensor_data = {
    'io_value': None,
    'ad_value': None
}


def sub_data_handler(sub_info):
    io_data, ad_data = sub_info
    print("io value: {0}, ad value: {1}".format(io_data, ad_data))

def is_path_clear():
    # สมมุติว่าค่า ad_value ที่มากกว่า 100 หมายถึงมีสิ่งกีดขวาง
    if sensor_data['ad_value'] is not None:
        
        return sensor_data['ad_value'] < 100
    
    return False

def wall_following_solve():
    while True:
        if is_path_clear():
            print('move')
        else:
            print('left')  # ถ้าทางเดินข้างหน้ามีสิ่งกีดขวาง ให้เลี้ยวซ้าย
            if is_path_clear():
                print('move')
            else:
                print('right')
                print('right')  # ถ้ายังไม่ว่าง ให้เลี้ยวขวา 2 ครั้งเพื่อกลับหลัง


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_sensor = ep_robot.sensor_adaptor
    ep_sensor.sub_adapter(freq=5, callback=sub_data_handler)
    time.sleep(2)
    try:
        wall_following_solve()
        
    except KeyboardInterrupt:
        print("Process interrupted")
    finally:
        ep_sensor.unsub_adapter()
        ep_robot.close()