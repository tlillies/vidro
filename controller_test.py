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
	
screen = curses.initscr()
screen.refresh()

vidro = Vidro(True, 115200,"127.0.0.1:14551")
vidro.connect()
controller = PositionController(vidro)

screen.clear()

while vidro.current_rc_channels[4] < 1600:
	
	controller.rc_alt(1000)
	
	curses_print("Throttle RC Level: " + str(vidro.current_rc_channels[2]), 6, 1)
	curses_print("Error: " + str(controller.error_alt), 7, 1)
	curses_print("Altitude:" + str(vidro.get_alt()), 8, 1)
	curses_print("T: "+ str(int(1370+controller.error_alt*controller.alt_K_P+controller.I_error_alt*controller.alt_K_I)) + " = 1370 + " + str(controller.error_alt*controller.alt_K_P) + " + " + str(controller.I_error_alt*controller.alt_K_I), 19, 0)
	
	
	
	#controller.rc_yaw()
	#controller.rc_xy()
	print vidro.current_rc_channels[4]
	vidro.get_mavlink()
vidro.close()
