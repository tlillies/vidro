import curses

global use_curses
use_curses = False

def curses_print(string, line):
	if use_curses == True:
		screen.addstr(line, 0, string)
			
	screen.refresh()
	
def connect_curses():
	global use_curses
	global screen
	use_curses = True
	screen = curses.initscr()
	screen.refresh()
	
connect_curses()
while True:
	curses_print("TEST", 0)
	curses_print("Test2", 1)
