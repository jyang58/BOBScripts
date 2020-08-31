"""

Path tracking simulation with pure pursuit steering control and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)

"""
import numpy as np
import math
import matplotlib.pyplot as plt
from gyro import *
from gpiozero import *
#from smbus2 import SMBus
from ros_path_planner import waypoints 
#%%
k = 0.1  # look forward gain
Lfc = 1.0  # look-ahead distance
Kp = 1.0  # speed propotional gain
dt = 0.1  # [s]
L = 2.9  # [m] wheel base of vehicle
#[5.5, 4.011546163136952, 2.0]
#[0.30000000000000004, 0.7820030965299936, 0.30000000000000004]
show_animation = True
#motor1 = Motor(17,18)
#motor2 = Motor(19,20)
#speed1 = 0
#speed2 = 0
#motor1.forward(speed1)
#motor2.forward(speed2)
for i in 
#accelerometer --> get speed
#know how fast speed --> something with angle
class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
    

def update(state, a, delta):
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    Vx = acc_x/16384.0 * dt
    Vy = acc_y/16384.0 * dt
    state.v = math.sqrt(Vx**2 + Vy**2)
    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt

    
    gyro_z = read_raw_data(GYRO_ZOUT_H)
    Gz = math.radians(dt*gyro_z/131.0)
    state.yaw = state.yaw + state.v / L * math.tan(Gz) * dt
    #torque, axel, wheel circumference
    target_v = state.v + a * dt
    #forward
    #delta
    delta_dependence = delta / math.pi()
    motor1.forward(max(min(target_v-delta_dependence),1),0)
    motor2.forward(max(min(target_v+delta_dependence),1),0)
    

    return state


def PIDControl(target, current):
    a = Kp * (target - current)

    return a


def pure_pursuit_control(state, cx, cy, pind):

    ind = calc_target_index(state, cx, cy)

    if pind >= ind:
        ind = pind

    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    if state.v < 0:  # back
        alpha = math.pi - alpha

    Lf = k * state.v + Lfc

    delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)

    return delta, ind


def calc_target_index(state, cx, cy):

    # search nearest point index
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d))
    L = 0.0

    Lf = k * state.v + Lfc

    # search look ahead target point index
    while Lf > L and (ind + 1) < len(cx):
        dx = cx[ind + 1] - cx[ind]
        dy = cx[ind + 1] - cx[ind]
        L += math.sqrt(dx ** 2 + dy ** 2)
        ind += 1

    return ind

