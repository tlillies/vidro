import curses 
from droneapi.lib import VehicleMode
from pymavlink import mavutil

api = local_connect()

v = api.get_vehicles()[0]

screen = curses.initscr()
dimensions = screen.getmaxyx()

vicon = curses.newwin(5, dimensions[1], 1, 0)
vicon.box()
vicon.addstr(0,0,"Vicon Data")

attitude = curses.newwin(5, dimensions[1], 6, 0)
attitude.box()
attitude.addstr(0,0,"Attitude Data")

armed = curses.newwin(5, 16, 11, 0)
armed.box()
armed.addstr(0,0,"Armed?")

vehicle_mode = curses.newwin(5, 16, 11, 16)
vehicle_mode.box()
vehicle_mode.addstr(0,0,"Mode:")

while True:

	screen.refresh()
	vicon.refresh()

	
	attitude.addstr(2,4, str(v.attitude))
	screen.refresh()
	attitude.refresh()


	armed.addstr(2,4,"Disarmed")
	screen.refresh()
	armed.refresh()


	vehicle_mode.addstr(2,6,"Mode")
	screen.refresh()
	vehicle_mode.refresh()

	screen.refresh()
	
screen.getch()	
curses.endwin()
