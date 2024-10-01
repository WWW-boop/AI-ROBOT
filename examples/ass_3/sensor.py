import robomaster
from robomaster import robot
import time


def sub_data_handler(sub_info):
    io_data, ad_data = sub_info
    print("io value: {0}, ad value: {1}".format(io_data, ad_data))
    # ตรวจสอบว่า IR sensor เจอกำแพงหรือไม่
    ir_front_left = io_data[1]  # เซ็นเซอร์หน้าซ้าย
    ir_left = io_data[0]  # เซ็นเซอร์ซ้าย
    ir_right = io_data[3]  # เซ็นเซอร์ขวา
    ir_front_right = io_data[2]  # เซ็นเซอร์หน้าขวา

    # เช็คระยะห่างจากกำแพง
    distance_left = ad_data[2]  # AD value 3: ระยะห่างทางซ้าย
    distance_right = ad_data[3]  # AD value 4: ระยะห่างทางขวา

    # แสดงผลข้อมูลที่ประมวลผล
    print(f"IR Sensors - Left: {ir_left}, Front Left: {ir_front_left}, Front Right: {ir_front_right}, Right: {ir_right}")
    print(f"Wall Distance - Left: {distance_left}, Right: {distance_right}")

    # ตรวจสอบว่ามีกำแพงอยู่ทางไหนบ้าง
    if ir_front_left == 0 and ir_left == 0:
        print('Wall left')

    if ir_front_right == 0 and ir_right == 0:
        print('Wall right')
    
    print("-" * 30)


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_sensor = ep_robot.sensor_adaptor
    ep_sensor.sub_adapter(freq=5, callback=sub_data_handler)  # เรียกใช้ callback ทุก 5 ครั้งต่อวินาที
    time.sleep(200)  # ใช้งานเซ็นเซอร์ 200 วินาที
    ep_sensor.unsub_adapter()
    ep_robot.close()
