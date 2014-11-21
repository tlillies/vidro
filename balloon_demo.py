#!/usr/bin/env python
# -*- coding: utf-8 -*-

from vidro import Vidro, ViconStreamer
from position_controller import PositionController
import sys, math, time
import socket, struct, threading
import curses
import matplotlib.pyplot as plot
import logging
import numpy as np
import cv2
import picamera
import picamera.array

def curses_print(string, line, col):
    """
    Function to do a simple curses print.
    """

    #Check for bad inputs
    if col > 1 or col < 0:
        return

    if line > 22 or line < 0:
        return

    #Print to screen using curses
    if col == 0:
        screen.addstr(line, 0, string)
    if col == 1:
        screen.addstr(line, 40, string)

    screen.refresh()
    
    
def camera(event):
    global frame
    global new_frame
    global plot_camera_time
    global Plot_camera_cycle
    global start_time
    previous_time = time.time()
    cycle = 0
    with picamera.PiCamera() as camera:
        with picamera.array.PiRGBArray(camera) as stream:
            camera.resolution = (320, 240)
            camera.hflip = True
            camera.vflip = True

            while not event.is_set():
                current_time = time.time()
                plot_camera_time.append(current_time-start_time)
                plot_camera_cycle.append(current_time-previous_time)
                plot_camera_cycle_number.append(cycle)
                cycle += 1
                previous_time = current_time
                curses_print("In camera loop",3,0)
                camera.capture(stream, 'bgr', use_video_port=True)
                # stream.array now contains the image data in BGR order
		frame = stream.array
                new_frame = True
                stream.truncate(0)
                
#Setup of vidro and controller
vidro = Vidro(False, 1)
flight_ready = vidro.connect()
controller = PositionController(vidro)

event = threading.Event()

d = threading.Thread(name='camera', target=camera, args=(event,))

d.start()

start_time = time.time()
previous_time = time.time()

cycle = 0

while (time.time()-start_time) < 15:
	
	controller.I_error_alt = 0
	controller.I_error_pitch = 0
	controller.I_error_roll = 0
	controller.I_error_yaw = 0

	controller.previous_time_alt = (time.time()-controller.timer)*10
	controller.previous_time_yaw = (time.time()-controller.timer)*10
	controller.previous_time_xy = (time.time()-controller.timer)*10

	vidro.previous_error_alt = 0
	vidro.previous_error_yaw = 0
	vidro.previous_error_roll = 0
	vidro.previous_error_pitch = 0

	if vidro.current_rc_channels[5] > 1600:
		controller.update_gains()
	
    current_time = time.time()
    curses_print(str(current_time-start_time),0,0)
    curses_print("New Frame: " + str(new_frame),7,0)
    plot_main_time.append(current_time-start_time)
    plot_main_cycle.append((current_time-previous_time))
    plot_main_cycle_number.append(cycle)
    cycle += 1
    previous_time = current_time
    if new_frame == True:
        """
        try:
            cv2.imshow('frame', frame)
        except:
            curses_print("Failed image show",1,0)
        """
        try:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            lower_blue = np.array([110, 50, 50], dtype=np.uint8)
            upper_blue = np.array([130,255,255], dtype=np.uint8)

            # Threshold the HSV image to get only blue colors
            mask = cv2.inRange(hsv, lower_blue, upper_blue)

            # Bitwise-AND mask and original image
            res = cv2.bitwise_and(frame,frame, mask= mask)
        
            #cv2.imshow('frame',frame)
            #cv2.imshow('mask',mask)
            cv2.imshow('res',res)
            new_frame = False
        except:
            pass

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    screen.clear()
    screen.refresh()

event.set()
d.join()

cv2.destroyAllWindows()
curses.endwin()
