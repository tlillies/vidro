from droneapi.lib import VehicleMode
from pymavlink import mavutil
import sys, math, time
import socket, struct, threading
import curses
from vicon_drone_curses import vicon_drone_curses

connect_droneapi()

set_sitl_home()

time.sleep(2)

timer = time.clock()

screen = curses.initscr()
screen.refresh()

while v.channel_readback['6'] == 1000:
	
	rc_go_to_alt(20000)
	rc_go_to_heading(2)
	screen.addstr(0,0, "Time: " + str(time.clock()-timer))
	screen.addstr(1,0, "X position: " + get_position()[0])
	screen.addstr(2,0, "Y position: " + get_position()[1])
	screen.addstr(3,0, "Z position: " + get_position()[2])
	time.sleep(.1)
	screen.refresh()
	
disarm()
