from vidro_class import Vidro, ViconStreamer
import sys, math, time
import socket, struct, threading
import curses
import utm
import matplotlib.pyplot as plot
from position_controller import PositionController

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

vidro = Vidro(True, 115200,"127.0.0.1:14551")
vidro.connect()
controller = PositionController(vidro)

screen = curses.initscr()
screen.clear()
screen.refresh()

while vidro.current_rc_channels[4] > 1600:

	controller.rc_alt(1000)
	controller.rc_yaw(0)
	controller.rc_xy(0,0)

	#Print alt data
	curses_print("Throttle RC Override: " + str(vidro.current_rc_overrides[2]), 5, 1)
	curses_print("Throttle RC Level: " + str(vidro.current_rc_channels[2]), 6, 1)
	curses_print("Error: " + str(controller.error_alt), 7, 1)
	curses_print("Altitude:" + str(vidro.get_alt()), 8, 1)
	curses_print("T: "+ str(int(1630+controller.error_alt*controller.alt_K_P+controller.I_error_alt*controller.alt_K_I)) + " = 1630 + " + str(controller.error_alt*controller.alt_K_P) + " + " + str(controller.I_error_alt*controller.alt_K_I), 19, 0)

	#Print yaw data
	curses_print("Yaw RC Level: " + str(vidro.current_rc_channels[3]), 6, 0)
	curses_print("Error: " + str(controller.error_yaw), 7, 0)
	curses_print("Heading Radians: " + str(vidro.get_yaw_radians()), 8, 0)
	curses_print("Heading Degrees: " + str(vidro.get_yaw_degrees()), 9, 0)
	curses_print("Y: "+ str(int(1500+controller.error_yaw*controller.yaw_K_P+controller.I_error_yaw*controller.yaw_K_I)) + " = 1500 + " + str(controller.error_yaw*controller.yaw_K_P) + " + " + str(controller.I_error_yaw*controller.yaw_K_I), 20, 0)

	#Print functions for curses
	curses_print("Pitch RC Level: " + str(vidro.current_rc_channels[1]), 11, 0)
	curses_print("Roll RC Level: " + str(vidro.current_rc_channels[0]), 11, 1)
	curses_print("Pitch: " + str(vidro.get_pitch()), 12, 0)
	curses_print("Roll: " + str(vidro.get_roll()), 12, 1)
	curses_print("X Error: " + str(round(controller.error_x)), 15, 0)
	curses_print("Y Error: " + str(round(controller.error_y)), 15, 1)
	curses_print("Roll Error: " + str(round(controller.error_roll)), 13, 1)
	curses_print("Pitch Error: " + str(round(controller.error_pitch)), 13, 0)
	#curses_print("Total Error: " + str(round(controller.total_error)), 16, 0)

	screen.refresh()
	time.sleep(.005)
	screen.clear()
	screen.refresh()

	vidro.get_mavlink()
vidro.get_mavlink()
vidro.close()
