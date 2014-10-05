#!/usr/bin/env python

import curses 

screen = curses.initscr()
#curses.use_color()

dimensions = screen.getmaxyx()

vicon = curses.newwin(5, dimensions[1], 0, 0)
#vicon.box()
vicon.addstr(0,0,"Vicon Data")
screen.refresh()
vicon.refresh()

attitude = curses.newwin(5, dimensions[1], 5, 0)
#attitude.box()
attitude.addstr(0,0,"Attitude Data")
screen.refresh()
attitude.refresh()

armed = curses.newwin(5, 16, 10, 0)
#armed.box()
armed.addstr(0,0,"Armed?")
armed.addstr(2,4,"Disarmed")
screen.refresh()
armed.refresh()

vehicle_mode = curses.newwin(5, 16, 10, 16)
#vehicle_mode.box()
vehicle_mode.addstr(0,0,"Mode:")
vehicle_mode.addstr(2,6,"Mode")
screen.refresh()
vehicle_mode.refresh()

screen.refresh()
screen.getch()
curses.endwin()
