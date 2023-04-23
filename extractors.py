import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys
import math

normal_speed = 1.0
turbo_speed = 0
accel_limit = 6.0
maxTurnRate = 0.25
commandInterval = 1.0/20
speed_to_erpm_gain = 5171
speed_to_erpm_offset = 180.0
erpm_speed_limit = 22000
steering_to_servo_gain = -0.9015
steering_to_servo_offset = 0.57
servo_min = 0.05
servo_max = 0.95
wheelbase = 0.324

def find_start_and_end_time(time, vels):

    start = -1
    end = -1
    starti, endi = 0, 0
    for i in range(vels.size):
        if vels[i] > 0.4:
            start = time[i]
            starti = i
            break
    for i in range(vels.size-1, -1, -1):
        if vels[i] > 0.4:
            end = time[i]
            endi = i
            break
    # print(end-start)
    return start, end, starti, endi

# def integrate(t, y):
#     res = [0.0000000001]
#     for i in range(1, t.size):
#         res.append(np.trapz(y[:i], x=t[:i]))

#     return np.array(res)

def extract_imu_data(filename):

    data_frame = pd.read_csv(filename)

    secs = data_frame["secs"].to_numpy()
    nsecs = data_frame["nsecs"].to_numpy()
    times = secs + nsecs / 1e9 - secs[0]

    # you need xyz of accelerometer and gyroscope
    # print keys
    print(data_frame.keys())
    imu_angular_vels = -(data_frame["z.1"].to_numpy())

    # imu_accel = np.column_stack((data_frame["x.2"].to_numpy(), data_frame["y.2"].to_numpy(), data_frame["z.2"].to_numpy()))
    # imu_gyro = np.column_stack((data_frame["x.1"].to_numpy(), data_frame["y.1"].to_numpy(), data_frame["z.1"].to_numpy()))
    
    return (times, imu_angular_vels)


def extract_joystick_data(subfolder):
    data_frame = pd.read_csv("./"+subfolder+"/_slash_joystick.csv")

    turbo_speed = 6.0

    secs = data_frame["secs"].to_numpy()
    nsecs = data_frame["nsecs"].to_numpy()
    joystick_times = secs + nsecs / 1e9 - secs[0]
    axes_strings = data_frame["axes"].to_numpy()
    axes = []
    for ax in axes_strings:
        ax = ax[1:-1]
        ax = ax.split(", ")
        ax = [float(a) for a in ax]
        axes.append(ax)
    axes = np.array(axes)

    steer_joystick = -axes[:, 0]
    drive_joystick = -axes[:, 4]

    turbo_mode = axes[:, 2] >= 0.9
    max_speed = turbo_speed
    # max_speed = turbo_mode * turbo_speed + (1 - turbo_mode) * normal_speed
    speed = drive_joystick * max_speed
    steering_angle = steer_joystick * maxTurnRate

    last_speed = 0.0
    clipped_speeds = []
    for s in speed:
        smooth_speed = max(s, last_speed - commandInterval*accel_limit)
        smooth_speed = min(smooth_speed, last_speed + commandInterval*accel_limit)
        last_speed = smooth_speed
        erpm = speed_to_erpm_gain * smooth_speed + speed_to_erpm_offset
        erpm_clipped = min(max(erpm, -erpm_speed_limit), erpm_speed_limit)
        clipped_speed = (erpm_clipped - speed_to_erpm_offset) / speed_to_erpm_gain
        clipped_speeds.append(clipped_speed)
    clipped_speeds = np.array(clipped_speeds)
    servo = steering_to_servo_gain * steering_angle + steering_to_servo_offset
    clipped_servo = np.fmin(np.fmax(servo, servo_min), servo_max)
    steering_angle = (clipped_servo - steering_to_servo_offset) / steering_to_servo_gain

    rot_vel = clipped_speeds / wheelbase * np.tan(steering_angle)

    return (joystick_times, clipped_speeds, rot_vel)