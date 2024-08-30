

from controller import Robot, DistanceSensor, Motor
import numpy as np

#-------------------------------------------------------
# Initialize variables

MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())   # [ms]
delta_t = robot.getBasicTimeStep()/1000.0    # [s]

# states
states = ['forward', 'turn_right', 'turn_left']
current_state = states[0]

# counter: used to maintain an active state for a number of cycles
counter = 0
COUNTER_MAX = 3

# Robot pose
# Adjust the initial values to match the initial robot pose in your simulation
x = -0.06    # position in x [m]
y = 0.436    # position in y [m]
phi = 0.0531  # orientation [rad]

# Robot velocity and acceleration
dx = 0.0   # speed in x [m/s]
dy = 0.0   # speed in y [m/s]
ddx = 0.0  # acceleration in x [m/s^2]
ddy = 0.0  # acceleration in y [m/s^2]

# Robot wheel speeds
wl = 0.0    # angular speed of the left wheel [rad/s]
wr = 0.0    # angular speed of the right wheel [rad/s]

# Robot linear and angular speeds
u = 0.0    # linear speed [m/s]
w = 0.0    # angular speed [rad/s]

# e-puck Physical parameters for the kinematics model (constants)
R = 0.0205    # radius of the wheels: 20.5mm [m]
D = 0.0565    # distance between the wheels: 52mm [m]
A = 0.05    # distance from the center of the wheels to the point of interest [m]


ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)


gs = []
gsNames = ['gs0', 'gs1', 'gs2']
for i in range(3):
    gs.append(robot.getDevice(gsNames[i]))
    gs[i].enable(timestep)


encoder = []
encoderNames = ['left wheel sensor', 'right wheel sensor']
for i in range(2):
    encoder.append(robot.getDevice(encoderNames[i]))
    encoder[i].enable(timestep)

oldEncoderValues = []

    
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)
 

def get_wheels_speed(encoderValues, oldEncoderValues, delta_t):
    
    wl = (encoderValues[0] - oldEncoderValues[0])/delta_t
    wr = (encoderValues[1] - oldEncoderValues[1])/delta_t

    return wl, wr


def get_robot_speeds(wl, wr, r, d):
    
    u = r/2.0 * (wr + wl)
    w = r/d * (wr - wl)

    return u, w


def get_robot_pose(u, w, x_old, y_old, phi_old, delta_t):
   
    delta_phi = w * delta_t
    phi = phi_old + delta_phi
    phi_avg = (phi_old + phi)/2   
    if phi >= np.pi:
        phi = phi - 2*np.pi
    elif phi < -np.pi:
        phi = phi + 2*np.pi
    
    delta_x = u * np.cos(phi_avg) * delta_t
    delta_y = u * np.sin(phi_avg) * delta_t
    x = x_old + delta_x
    y = y_old + delta_y

    return x, y, phi


  
def get_robot_displacement(encoderValues, oldEncoderValues, r, d):
    
    dl = (encoderValues[0] - oldEncoderValues[0])*r
    dr = (encoderValues[1] - oldEncoderValues[1])*r
    
    lin_disp = (dr + dl)/2.0   
    ang_disp = (dr - dl)/d      

    return lin_disp, ang_disp


def get_robot_pose2(lin_disp, ang_disp, x_old, y_old, phi_old):
    """Updates robot pose based on heading and displacement"""
    phi = phi_old + ang_disp
    phi_avg = (phi_old + phi)/2.0   
    if phi >= np.pi:
        phi = phi - 2*np.pi
    elif phi < -np.pi:
        phi = phi + 2*np.pi
    
    delta_x = lin_disp * np.cos(phi_avg)
    delta_y = lin_disp * np.sin(phi_avg)
    x = x_old + delta_x
    y = y_old + delta_y

    return x, y, phi


while robot.step(timestep) != -1:
    
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    gsValues = []
    for i in range(3):
        gsValues.append(gs[i].getValue())

    encoderValues = []
    for i in range(2):
        encoderValues.append(encoder[i].getValue())    
        
   
    if len(oldEncoderValues) < 2:
        for i in range(2):
            oldEncoderValues.append(encoder[i].getValue())   


    line_right = gsValues[0] > 600
    line_left = gsValues[2] > 600


    if current_state == 'forward':
       
        leftSpeed = MAX_SPEED
        rightSpeed = MAX_SPEED

        if line_right and not line_left:
            current_state = 'turn_right'
            counter = 0
        elif line_left and not line_right:
            current_state = 'turn_left'
            counter = 0
            
    if current_state == 'turn_right':
      
        leftSpeed = 0.8 * MAX_SPEED
        rightSpeed = 0.4 * MAX_SPEED

        if counter == COUNTER_MAX:
            current_state = 'forward'

    if current_state == 'turn_left':
        leftSpeed = 0.4 * MAX_SPEED
        rightSpeed = 0.8 * MAX_SPEED

        if counter == COUNTER_MAX:
            current_state = 'forward'        

    counter += 1

    [wl, wr] = get_wheels_speed(encoderValues, oldEncoderValues, delta_t)

    [u, w] = get_robot_speeds(wl, wr, R, D)
    

    [x, y, phi] = get_robot_pose(u, w, x, y, phi, delta_t)

    
    oldEncoderValues = encoderValues
    
    print(f'Sim time: {robot.getTime():.3f}  Pose: x={x:.2f} m, y={y:.2f} m, phi={phi:.4f} rad.')


    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

  
