import robomaster
from robomaster import robot
import time

class RoboMasterMazeSolver:
    def __init__(self):
        self.ep_robot = robot.Robot()
        self.ep_robot.initialize(conn_type="ap")
        self.ep_sensor = self.ep_robot.sensor_adaptor
        self.ep_gimbal = self.ep_robot.gimbal
        self.ep_tof = self.ep_robot.sensor

        self.tof_distance = None
        self.dis_ssL = None
        self.dis_ssR = None

    def log_movement(self, action, direction):
        print(f"Action: {action}, Direction: {direction}")

    def move_forward(self):
        self.log_movement("move_forward", "forward")
        self.ep_robot.chassis.move(x=0.1, y=0, z=0, xy_speed=0.5).wait_for_completed()

    def turn_left(self):
        self.log_movement("turn_left", "left")
        self.ep_robot.chassis.move(x=0, y=0, z=90, z_speed=30).wait_for_completed()

    def turn_right(self):
        self.log_movement("turn_right", "right")
        self.ep_robot.chassis.move(x=0, y=0, z=-90, z_speed=30).wait_for_completed()

    def tof_data_handler(self, sub_info):
        self.tof_distance = sub_info[0]
        print("tof1: {0}".format(self.tof_distance))

    def sub_data_handler(self, sub_info):
        io_data, ad_data = sub_info

        ssR = ad_data[2]
        ssL = ad_data[3]

        vaR, vaL = self.convert_to_V(ssR, ssL)
        self.dis_ssR = self.convert_to_cm(vaR)
        self.dis_ssL = self.convert_to_cm(vaL)

        print(f"Distance ssR: {self.dis_ssR} cm")
        print(f"Distance ssL: {self.dis_ssL} cm")

    def convert_to_V(self, ssR, ssL):
        ad_data_vo_ssr = (ssR * 3) / 1023
        ad_data_vo_ssl = (ssL * 3) / 1023
        return ad_data_vo_ssr, ad_data_vo_ssl

    def convert_to_cm(self, voltage):
        if voltage > 1.4:
            cm = (voltage - 4.2) / -0.31
        elif 1.4 >= voltage >= 0.6:
            cm = (voltage - 2.03) / -0.07
        else:
            cm = (voltage - 0.95) / -0.016
        return cm

    def front_wall(self):
        return self.tof_distance is not None and self.tof_distance < 230

    def left_wall(self):
        return self.dis_ssL is not None and self.dis_ssL < 25

    def right_wall(self):
        return self.dis_ssR is not None and self.dis_ssR < 40

    def stick_right_wall(self):
        if self.dis_ssR is not None:
            err_dis_r = (self.dis_ssR - 20) / 100
            if abs(err_dis_r) >= 0.1:
                self.ep_robot.chassis.move(x=0, y=err_dis_r, z=0, xy_speed=1).wait_for_completed()
            return True
        return False

    def wall_following_solve(self):
        while True:
            time.sleep(0.5)
            if self.right_wall():
                self.stick_right_wall()
                if not self.front_wall():
                    self.move_forward()
                else:
                    self.turn_left()
            else:
                for i in range(3):
                    self.move_forward()
                self.turn_right()
                self.ep_gimbal.recenter().wait_for_completed()
                for i in range(7):
                    self.move_forward()
                if not self.right_wall():
                    self.turn_right()
                    self.ep_gimbal.recenter().wait_for_completed()
                    for i in range(7):
                        self.move_forward()

            self.ep_gimbal.recenter().wait_for_completed()

    def start(self):
        self.ep_gimbal.recenter().wait_for_completed()
        self.ep_sensor.sub_adapter(freq=5, callback=self.sub_data_handler)
        self.ep_tof.sub_distance(freq=5, callback=self.tof_data_handler)

        try:
            self.wall_following_solve()
        except KeyboardInterrupt:
            print("Process interrupted")
        finally:
            self.ep_sensor.unsub_adapter()
            self.ep_tof.unsub_adapter()
            self.ep_robot.close()

if __name__ == '__main__':
    solver = RoboMasterMazeSolver()
    solver.start()