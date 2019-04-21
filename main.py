# coding: utf-8

from GY_85 import GY85

import time
from math import *
import sys

import curses

###############################################
#Start Vpython:

if len(sys.argv) > 1 and sys.argv[1] == "--v":
    from vpython import *
    
    scene = canvas(title = 'IMU Visualizer',\
            x = 0, y = 0, width=400, height = 400,\
            scale = vector(1.2, 1.2, 1.2), \
            center = vector(0, 0, 0), \
            forward = vector(1, 0, -0.8), \
            up = vector(0, 0, 1), \
            background = vector(0, 0, 0))
    
    mybox = box(length = 1, height = 0.2, width = 1,\
                axis = vector(1, 0, 0), \
                up = vector(0, 0, 1), \
                color = color.green)

    #my_arrow = arrow(pos = vector(0, 0, 0.2), axis= vector(1, 0, 0), \
    #            shaftwidth = 0, color = color.red)
    
#################################################
# Init Sensor:
sensor = GY85()

grad2rad = pi / 180.0
rad2grad = 1 / grad2rad

def process_accel(accel, a_mem):
    # Turn raw accel values in G:
    G_range = 4     #default value = 4
    resolution = 10 #default value = 10
    k = (G_range/(2**resolution))
    processed = [val * k for val in accel]
    # Low pass filter:
    alpha = 0.5
    filtered = [val * alpha + (a_mem[i] * (1.0 - alpha)) for i, val in enumerate(processed)]
    # Calc Roll and Pitch:
    rad2grad = 180.0 / pi
    roll = (atan2(filtered[1], filtered[2])) * rad2grad
    pitch = (atan2(-filtered[0], sqrt(filtered[1]**2 + filtered[2]**2))) * rad2grad
    return (filtered, [roll, pitch])

def process_gyro(gyro, g_mem, deltat):
    # Sensitivity scale factor:
    ssf = 14.375
    processed = [val / ssf for val in gyro]
    # Calc angles
    g_mem[0] += processed[0] * deltat
    g_mem[1] += processed[1] * deltat
    g_mem[2] += processed[2] * deltat
    return (processed, g_mem)
    
def update_box(vect):
    roll = vect[0] * grad2rad
    pitch = vect[1] * grad2rad
    yaw = -vect[2] * grad2rad
    # Convert (roll, pitch, yaw) to (axis, up)
    axis=vector(cos(pitch)*cos(yaw), \
                -cos(pitch)*sin(yaw), \
                sin(pitch))
    up=vector(sin(roll)*sin(yaw)-cos(roll)*sin(pitch)*cos(yaw), \
              sin(roll)*cos(yaw)+cos(roll)*sin(pitch)*sin(yaw), \
              cos(roll)*cos(pitch))
    #update box pos
    mybox.axis = axis
    mybox.up = up

def update_arrow(vect):
    my_arrow.axis = vector(vect[0], vect[1], 0)
    
    
if __name__ == "__main__":
    acc_x_str = "Acc_X : "
    acc_y_str = "Acc_Y : "
    acc_z_str = "Acc_Z : "
    gyro_roll_str = "Roll : "
    gyro_pitch_str = "Pitch : "
    gyro_yaw_str = "Yaw : "
    temp_str = "Temperature (°C) : "
    heading_str = "Heading : "
    
    stdscr = curses.initscr()
    curses.noecho()
    stdscr.nodelay(True)

    offset = 32
    
    stdscr.addstr(0, 0, acc_x_str)
    stdscr.addstr(1, 0, acc_y_str)
    stdscr.addstr(2, 0, acc_z_str)
    stdscr.addstr(0, offset, gyro_roll_str)
    stdscr.addstr(1, offset, gyro_pitch_str)
    stdscr.addstr(2, offset, gyro_yaw_str)
    stdscr.addstr(4, 0, heading_str)
    stdscr.addstr(4, offset, temp_str)
    stdscr.addstr(6, 8, "[ Press 'q' to exit ]")
    
    last = time.time()
    processed_accel = [0, 0, 0]
    g_vect = [0, 0, 0]

    while True:
        
        # Grab datas:
        data = sensor.read()
        deltat = time.time() - last
        last = time.time()
        
        # Process datas:
        processed_accel, a_vect = process_accel(data[1], processed_accel)
        processed_gyro, g_vect = process_gyro(data[0], a_vect + [0], deltat)
        
        # Combine datas:
        ratio = 0.98
        g_vect[0] = ratio * (g_vect[0]) + (1 - ratio) * a_vect[0]
        g_vect[1] = ratio * (g_vect[1]) + (1 - ratio) * a_vect[1]
        #g_vect[2] = 0
        
        #update box or print datas:
        if len(sys.argv) > 1 and "--v" in sys.argv:
            update_box(g_vect)
            #update_arrow(g_vect)

        stdscr.addstr(0, len(acc_x_str), str(round(processed_accel[0], 2)) + "  ")
        stdscr.addstr(1, len(acc_y_str), str(round(processed_accel[1], 2)) + "  ")
        stdscr.addstr(2, len(acc_z_str), str(round(processed_accel[2], 2)) + "  ")

        stdscr.addstr(0, offset + len(gyro_roll_str), str(round(g_vect[0], 2)) + "  ")
        stdscr.addstr(1, offset + len(gyro_pitch_str), str(round(g_vect[1], 2)) + "  ")
        stdscr.addstr(2, offset + len(gyro_yaw_str), str(round(g_vect[2], 2)) + "  ")

        additional_data = sensor.magneto_read()
        
        stdscr.addstr(4, len(heading_str), str(round(data[2], 2)) + "  ")
        stdscr.addstr(4, offset + len(temp_str), str(round(additional_data[1], 2)) + "  ")
      
        time.sleep(0.05)
        
        c = stdscr.getch()
        if c == ord('q'):
            break  # Exit the while loop
        
    curses.endwin()
    exit()
