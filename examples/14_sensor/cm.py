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





def sub_data_handler(sub_info):
    io_data, ad_data = sub_info
    global ad_data_value
    global dis_ssL
    global dis_ssR
    ad_data_value = ad_data
    ssR = ad_data_value[2]
    ssL = ad_data_value[3]

    vaR, vaL = convert_to_V(ssR, ssL)
    dis_ssR = convert_to_cm(vaR)
    dis_ssL = convert_to_cm(vaL)
    print(f"Distance ssR: {dis_ssR}  cm")
    print(f"Distance ssL: {dis_ssL}  cm")

def convert_to_V(ssR,ssL):
    # Assuming the sensor value (ssR) needs to be converted to centimeters.
    ad_data_vo_ssr = (ssR * 3) / 1023
    ad_data_vo_ssl = (ssL * 3) / 1023
    return ad_data_vo_ssr , ad_data_vo_ssl

def convert_to_cm(voltage):
    if voltage > 1.4:
        cm = (voltage - 4.2) / -0.31
    elif 1.4 >= voltage >= 0.6:
        cm = (voltage - 2.03) / -0.07
    else:
        cm = (voltage - 0.95) / -0.016
    
    return cm




if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_sensor = ep_robot.sensor_adaptor
    ep_sensor.sub_adapter(freq=5, callback=sub_data_handler)
    time.sleep(60)
    ep_sensor.unsub_adapter()
    ep_robot.close()
