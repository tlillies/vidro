from vidro_class import Vidro, ViconStreamer
from position_controller import PositionController
import sys, math, time
import socket, struct, threading
import curses
import utm
import matplotlib.pyplot as plot

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

vidro = Vidro(False, 57600, '/dev/ttyUSB0')
vidro.connect()
controller = PositionController(vidro)

screen = curses.initscr()
screen.clear()
screen.refresh()

while True:

	screen.erase()
	if v.channel_readback['1'] < RC_roll_min:
		RC_roll_min = v.channel_readback['1']
	if v.channel_readback['1'] > RC_roll_max:
		RC_roll_max = v.channel_readback['1']

	if v.channel_readback['2'] < RC_pitch_min:
		RC_pitch_min = v.channel_readback['2']
	if v.channel_readback['2'] > RC_pitch_max:
		RC_pitch_max = v.channel_readback['2']

	if v.channel_readback['3'] < RC_throttle_min:
		RC_throttle_min = v.channel_readback['3']
	if v.channel_readback['3'] > RC_throttle_max:
		RC_throttle_max = v.channel_readback['3']

	if v.channel_readback['4'] < RC_yaw_min:
		RC_yaw_min = v.channel_readback['4']
	if v.channel_readback['4'] > RC_yaw_max:
		RC_yaw_max = v.channel_readback['4']

	curses_print("Roll: " + str(v.channel_readback['1']), 0,0)
	curses_print("Pitch: " + str(v.channel_readback['2']), 1,0)
	curses_print("Throttle: " + str(v.channel_readback['3']), 2,0)
	curses_print("Yaw: " + str(v.channel_readback['4']), 3,0)
	time.sleep(.1)

	screen.refresh()
	time.sleep(.02)
	screen.clear()
	screen.refresh()

	vidro.get_mavlink()

vidro.close()
