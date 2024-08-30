import robomaster
from robomaster import robot
import time
import keyboard
import matplotlib.pyplot as plt
time_tof = []
time_all = []
s_r_after = []
s_l_after = []
r_fill= []
s_l = []
s_r = []
adc_l_fill = []
adc_r_fill = []
axis_x = []
axis_y = []
lst_adap = []
lst_tof = []
tof_fill = []
lst_io = []
f_way = []
all_p_data = []
dict_w = {}
speed = 35
direc = None
current_x = None
current_y = None
start_time = time.time()
t_sharp = []



def filter_signal(sen_l, sen_r):
    all_sensor=[s_l, s_r]
    sen = [sen_l, sen_r]
    res=[]
    for i in range(len(sen)):
        yn = sen[i] + (0.1*all_sensor[i][-1])
        res.append(yn)
    adc_l_fill.append(res[0])
    adc_r_fill.append(res[1])
    
    return res[0], res[1]

def filter_tof(now):
    yn = now + (0.1*lst_tof[-1])
    return yn

def calculator_adc_f(sen_l, sen_r):
    if len(s_r_after) > 1 and len(s_l_after) > 1 :
        sen_l, sen_r = filter_signal(sen_l, sen_r)
    res = []
    volt_l = (sen_l/1023)*3
    volt_r = (sen_r/1023)*3
    sensor = [volt_l, volt_r]
    for i in sensor:
        #err 2 cm -> 8 cm
        if i >= 1.6:
            cm = (i-4.2)/-0.326
            res.append(cm-2)
        #err 3 cm -> 14 cm
        elif i >= 0.73:
            # print('3')
            cm = (i-2.7)/-0.1375
            res.append(cm-3)
        elif i >= 0.25:
            # print('3')
            cm = (i-1.67)/-0.0517
            res.append(cm-3)
        else :
            res.append(1)
    # print(sensor)
    # print(res)
    if res :
        return res[0], res[1]
    

def sub_position_handler(position_info):
    x, y, z = position_info
    axis_x.append(x)
    axis_y.append(y)
    now_time = time.time() - start_time
    time_all.append(now_time)
    # print(len(axis_x), len(axis_y), len(time_all))

def sub_gimbal(angle_info):
    global list_of_data
    list_of_data = angle_info

def sub_data_handler(distance):
    now = distance[0]
    lst_tof.append(now)
    if len(lst_tof) > 1:
        yn = filter_tof(now)
    tof_fill.append(yn)
    time_tof.append(time.time() - start_time)
    print('tof', len(tof_fill), len(time_tof), len(lst_tof))
    # print(lst_tof[-1])

def sub_adapter(sub_adaptor):
    io_data, adap = sub_adaptor
    lst_adap.append(adap)
    s_l.append(lst_adap[1][2])
    s_r.append(lst_adap[1][1])  
    if lst_adap:
        l1, r1 = calculator_adc_f(lst_adap[1][2], lst_adap[1][1])
        s_l_after.append(l1)
        s_r_after.append(r1)
        t_sharp.append(time.time() - start_time)
    # print('info left : {} | right : {} | back : {}'.format(info[1], info[2], info[0]))

def rotation(vector):
    if type(vector) == int :
        ep_chasis.move(x=0, y=0, z=vector, z_speed=90).wait_for_completed()


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chasis = ep_robot.chassis
    ep_gimbal = ep_robot.gimbal
    ep_sensor = ep_robot.sensor
    ep_sensor_adaptor = ep_robot.sensor_adaptor


    #Gimbal
    ep_gimbal.sub_angle(freq=50, callback=sub_gimbal)
    #TOF
    ep_sensor.sub_distance(freq=50, callback=sub_data_handler)
    #ADP
    ep_sensor_adaptor.sub_adapter(freq=50, callback=sub_adapter)
    #chasis
    ep_chasis.sub_position(freq=50, callback=sub_position_handler)


    count = 0
    data = None
    state = None
    parts = None
    deg = [-90, 90, 0]
    wall = None
    target = None
    ep_gimbal.recenter(pitch_speed=200, yaw_speed=200).wait_for_completed()
    counter = 0

    while True:
            print(len(adc_r_fill))
            print(len(adc_l_fill))
            if lst_tof:
                targetx = axis_x[0]
                targety = axis_y[0]

            if s_r_after[-1] > 2 and lst_tof[-1] < 300 and s_l_after[-1] > 2 and counter == 0:
                ep_chasis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
                time.sleep(0.2)
                rotation(180)
                counter += 1
                time.sleep(0.2)
            if counter and abs(targetx - axis_x[-1]) <= 0.1 and abs(targety - axis_y[-1]) <= 0.1:
                ep_chasis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
                ep_sensor_adaptor.unsub_adapter()
                ep_chasis.unsub_position()
                ep_sensor.unsub_distance()
                ep_robot.close()
                break
            else :
                ep_chasis.drive_wheels(w1=speed, w2=speed, w3=speed, w4=speed)

    

    plt.figure(figsize=(10, 8))
    plt.subplot(3, 2, 1)
    plt.plot(time_all, axis_x, label='distance', marker='*', color='blue')
    plt.ylabel('X Position')
    plt.xlabel('Time')
    plt.legend()

    plt.subplot(3, 2, 2)
    plt.plot(time_all, axis_y, label='distance', marker='*', color='blue')
    plt.xlabel('Time')
    plt.ylabel('Y Position')
    plt.legend()

    plt.subplot(3, 2, 3)
    plt.plot(time_tof, [tof_fill[i] for i in range(len(time_tof))], label='after', marker='*', color='orange')
    plt.plot(time_tof, [lst_tof[i] for i in range(len(time_tof))], label='before', marker='*', color='black')
    plt.xlabel('Time')
    plt.ylabel('TOF sensor')
    plt.legend()

    plt.subplot(3, 2, 4)
    plt.plot([t_sharp[i] for i in range(len(s_l))], s_l, label='after', marker='*', color='orange')
    plt.plot([t_sharp[i] for i in range(len(adc_l_fill))], adc_l_fill, label='before', marker='*', color='black')
    plt.xlabel('Time')
    plt.ylabel('Sharp sensor Left')
    plt.legend()

    plt.subplot(3, 2, 5)
    plt.plot([t_sharp[i] for i in range(len(s_r))], s_r, label='after', marker='o', color='orange')
    plt.plot([t_sharp[i] for i in range(len(adc_r_fill))], adc_r_fill, label='before', marker='o', color='black')
    plt.xlabel('Time')
    plt.ylabel('Sharp sensor Right')
    plt.legend()
    plt.show()