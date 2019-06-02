# coding: utf-8

from GY_85 import GY85
import time
from math import *
import sys
import curses
from vpython import *
import csv

'''
IMU Sensor placed Y in front

'''

# Init Sensor:
sensor = GY85()
# Init tools:
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
    roll = (atan2(-filtered[1], filtered[2])) * rad2grad
    pitch = (atan2(filtered[0], sqrt(filtered[1]**2 + filtered[2]**2))) * rad2grad
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

def init_curses():
    str_dict = {
        "acc_x_str" : "Acc_X : ",
        "acc_y_str" : "Acc_Y : ",
        "acc_z_str" : "Acc_Z : ",
        "gyro_roll_str" : "Roll : ",
        "gyro_pitch_str" : "Pitch : ",
        "gyro_yaw_str" : "Yaw : ",
        "temp_str" : "Temperature (°C) : ",
        "heading_str" : "Heading : ",
        "exit_str" : "[ Press 'q' to exit ]"}
    stdscr = curses.initscr()
    curses.noecho()
    stdscr.nodelay(True)
    offset = 32
    stdscr.addstr(0, 0, str_dict["acc_x_str"])
    stdscr.addstr(1, 0, str_dict["acc_y_str"])
    stdscr.addstr(2, 0, str_dict["acc_z_str"])
    stdscr.addstr(0, offset, str_dict["gyro_pitch_str"])
    stdscr.addstr(1, offset, str_dict["gyro_roll_str"])
    stdscr.addstr(2, offset, str_dict["gyro_yaw_str"])
    stdscr.addstr(4, 0, str_dict["heading_str"])
    stdscr.addstr(4, offset, str_dict["temp_str"])
    stdscr.addstr(6, 8, str_dict["exit_str"])
    return (stdscr, str_dict, offset)

def update_curses(stdscr, str_dict, offset, processed_accel, g_vect, additional_data):
    stdscr.addstr(0, len(str_dict["acc_x_str"]), str(round(processed_accel[0], 2)) + "  ")
    stdscr.addstr(1, len(str_dict["acc_y_str"]), str(round(processed_accel[1], 2)) + "  ")
    stdscr.addstr(2, len(str_dict["acc_z_str"]), str(round(processed_accel[2], 2)) + "  ")
    stdscr.addstr(0, offset + len(str_dict["gyro_pitch_str"]), str(round(g_vect[0], 2)) + "  ")
    stdscr.addstr(1, offset + len(str_dict["gyro_roll_str"]), str(round(g_vect[1], 2)) + "  ")
    stdscr.addstr(2, offset + len(str_dict["gyro_yaw_str"]), str(round(g_vect[2], 2)) + "  ")
    stdscr.addstr(4, len(str_dict["heading_str"]), str(round(additional_data[2], 2)) + "  ")
    stdscr.addstr(4, offset + len(str_dict["temp_str"]), str(round(additional_data[1], 2)) + "  ")
    c = stdscr.getch()
    if c == ord('q'):
        curses.endwin()
        return (1)
    else:
        return (0)

def init_vpython():
    scene = canvas(title = 'IMU Visualizer',\
                x = 0, y = 0, width=400, height = 400,\
                scale = vector(1.2, 1.2, 1.2), \
                center = vector(0, 0, 0), \
                forward = vector(1, 0, 0), \
                up = vector(0, 0, 1), \
                background = vector(0, 0, 0))
    my_box = box(length = 1, height = 0.2, width = 1,\
                axis = vector(1, 0, 0), \
                up = vector(0, 0, 1), \
                color = color.green)
    return (my_box)

def update_vpython(vect, mybox):
    pitch = vect[0] * grad2rad
    roll = vect[1] * grad2rad
    yaw = vect[2] * grad2rad
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
    
if __name__ == "__main__":
    # Parse args
    for i, arg in enumerate(sys.argv):
        if arg == "--o":
            if len(sys.argv) > i:
                output_file = sys.argv[i + 1]
                # Write first line
                with open(output_file, mode="w") as f:
                    writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                    writer.writerow(["AccX",
                                     "AccY",
                                     "AccZ",
                                     "GyroX",
                                     "GyroY",
                                     "GyroZ",
                                     "Heading"])
        if arg == "--r":
            if len(sys.argv) > i:
                input_file = sys.argv[i + 1]
                file_data = []
                # read file
                with open(input_file, mode='r') as f:
                    reader = csv.reader(f, delimiter=',')
                    line_count = 0
                    for line in reader:
                        if line_count > 0:
                            # cast to float
                            for i, elem in enumerate(line):
                                line[i] = float(elem)
                            file_data.append(((line[0], line[1], line[2]),
                                             (line[3], line[4], line[5])))
                        line_count += 1
                line_count = 0
        if arg == "--v":
            # Init vpython window
            my_box = init_vpython()
    if "--v" not in sys.argv:
        # Init curses window
        stdscr, str_dict, offset = init_curses()
        
    # Start
    last = time.time()
    processed_accel = [0, 0, 0]
    g_vect = [0, 0, 0]

    while True:
        # Grab datas:
        if "--r" in sys.argv:
            if line_count < len(file_data):
                data = file_data[line_count]
            else:
                if "--v" not in sys.argv:
                    curses.endwin()
                break
            line_count += 1
        else:
            data = sensor.read()
        additional_data = sensor.magneto_read()
        deltat = time.time() - last
        last = time.time()
        # Process datas:
        processed_accel, a_vect = process_accel(data[0], processed_accel)
        processed_gyro, g_vect = process_gyro(data[1], a_vect + [0], deltat)
        # Combine datas and adjust signs:
        ratio = 0.98
        g_vect[0] = -1 * (ratio * (g_vect[0]) + (1 - ratio) * a_vect[0])
        g_vect[1] = ratio * (g_vect[1]) + (1 - ratio) * a_vect[1]
        g_vect[2] = -1 * g_vect[2]
        # Save data
        if "--o" in sys.argv:
            with open(output_file, mode="a") as f:
                writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                writer.writerow([str(round(processed_accel[0], 2)),
                                 str(round(processed_accel[1], 2)),
                                 str(round(processed_accel[2], 2)),
                                 str(round(processed_gyro[0], 2)),
                                 str(round(processed_gyro[1], 2)),
                                 str(round(processed_gyro[2], 2)),
                                 str(round(additional_data[2], 2))])
        # Update box or print datas:
        if "--v" in sys.argv:
            update_vpython(g_vect, my_box)
        else:
            if update_curses(stdscr, str_dict, offset, 
                            processed_accel, g_vect,
                            additional_data):
                break
        time.sleep(0.01)
    exit(0)
