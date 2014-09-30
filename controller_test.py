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

#screen = curses.initscr()
#screen.clear()
#screen.refresh()

while vidro.current_rc_channels[4] > 1600:
	
	controller.rc_alt(1000)
	
	#curses_print("Throttle RC Override: " + str(vidro.current_rc_overrides[2]), 5, 1)
	#curses_print("Throttle RC Level: " + str(vidro.current_rc_channels[2]), 6, 1)
	#curses_print("Error: " + str(controller.error_alt), 7, 1)
	#curses_print("Altitude:" + str(vidro.get_alt()), 8, 1)
	#curses_print("T: "+ str(int(1630+controller.error_alt*controller.alt_K_P+controller.I_error_alt*controller.alt_K_I)) + " = 1630 + " + str(controller.error_alt*controller.alt_K_P) + " + " + str(controller.I_error_alt*controller.alt_K_I), 19, 0)
	#screen.refresh()
	#screen.clear()
	time.sleep(.005)
	
	#controller.rc_yaw()
	#controller.rc_xy()
vidro.get_mavlink()
vidro.close()
