from droneapi.lib import VehicleMode
from pymavlink import mavutil
import sys, math, time
import socket, struct, threading
import curses
import utm
import matplotlib.pyplot as plot

#Global variable for determining whether to use SITL or Vicon
sitl = True


#Global variable for determining whether to use curses or not
use_curses = False


#Home x,y,x position
home_x = 0
home_y = 0
home_z = 0


#Home lat,lon, and alt for sitl
home_lat = 0
home_lon = 0
home_alt = 0


#Fence for safety (Not implemented yet)
fence_x_min = 0
fence_x_max = 0
fance_y_min = 0
fence_y_max = 0
fence_z_min = 0
fence_z_max = 0


#Previous errors for calculating I and D
previous_time_alt = 0
I_error_alt = 0
error_alt = 0

previous_time_yaw = 0
I_error_yaw = 0
error_yaw = 0

previous_time_xy = 0
previous_error_pitch = 0
previous_error_roll = 0
D_error_roll = 0
D_error_pitch = 0
I_error_roll = 0
I_error_pitch = 0
error_pitch = 0
error_roll = 0
error_x = 0
error_y = 0


#Used for averaging out lat-lon to make smoother input
sum_lat = 0
sum_lon = 0
count_lat_lon = 0


#Gains for PID controller
alt_K_P = 0
alt_K_I = 0

yaw_K_P = 0
yaw_K_I = 0

roll_K_P = 0
roll_K_I = 0
roll_K_D = 0

pitch_K_P = 0
pitch_K_I = 0
pitch_K_D = 0


#Current x and y position (For Plottting)
x_current = 0
y_current = 0


#Plot arrays to start previous data fro plotting
plot_error_yaw=[]
plot_error_yaw_I=[]
plot_time_yaw=[]

plot_error_throttle=[]
plot_error_throttle_I=[]
plot_time_throttle=[]

plot_error_pitch=[]
plot_error_pitch_I=[]
plot_error_pitch_D = []
plot_time_pitch=[]
plot_rc_pitch=[]

plot_error_roll=[]
plot_error_roll_I=[]
plot_error_roll_D = []
plot_time_roll=[]
plot_rc_roll=[]

plot_x_current=[]
plot_y_current=[]


#ViconStreamer class from: Cameron Finucane <cpf37@cornell.edu>
class ViconStreamer:
	# based on example from http://docs.python.org/howto/sockets.html

	def __init__(self):
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self._streamNames = None
		self._desiredStreams = []
		self._streaming = False
		self._verbose = False
		self.data = None

	def connect(self, host, port):
		print ">> Connecting..."
		self.sock.connect((host, port))
		print ">> Requesting stream info..."
		# Mysteriously, the ordering of the following two bytes is the opposite
		# of what the Vicon documentation claims it is
		self._viconSend([1,0])
		print ">> Receiving stream info..."
		self._streamNames = s._viconReceive()

	def _send(self, msg):
		totalsent = 0
		while totalsent < len(msg):
			sent = self.sock.send(msg[totalsent:])
			if sent == 0:
				raise RuntimeError("socket connection broken")
			totalsent = totalsent + sent

	def close(self):
		self.stopStreams()

		print ">> Disconnecting..."
		self.sock.close()

	def getData(self):
		# returns None if no data is available yet

		if self.data is None:
			return None

		return [self.data[i] for i in self._desiredStreams]

	def printStreamInfo(self):
		if self._streamNames is None:
			raise RuntimeError("stream info is not available because you are not connected")
		else:
			print "Available streams:"
			print "		" + "\n	".join(["(%d) %s" % (i,n) for i,n in enumerate(self._streamNames)])

	def selectStreams(self, names):
		# For each name passed in, finds and subscribes to all streams whose name contains
		# the input name.
		# Returns the full stream names.

		if self._streamNames is None:
			raise RuntimeError("cannot set streams because you are not connected")

		matchingStreamNames = []
		self._desiredStreams = []
		for m in names:
			found = False
			for i,n in enumerate(self._streamNames):
				if m in n:
					matchingStreamNames.append(n)
					self._desiredStreams.append(i)
					found = True
			if not found:
				raise RuntimeError("could not find stream matching name '%s'" % m)

		print ">> Subscribed to streams: " + ", ".join(matchingStreamNames)
		return matchingStreamNames

	def startStreams(self, verbose=False):
		if self._desiredStreams == []:
			raise RuntimeError("cannot start streaming because no streams are selected")

		print ">> Starting streams..."
		self._viconSend([3,0])

		self._verbose = verbose
		self._streaming = True
		self.listenThread = threading.Thread(target = self._processStream)
		self.listenThread.start()

	def stopStreams(self):
		if not self._streaming:
			return

		print ">> Stopping streams..."
		self._streaming = False
		self.listenThread.join()

		self._viconSend([4,0])

	def _viconSend(self, data):
		msg = struct.pack('<' + str(len(data)) + 'L', *data)
		self._send(msg)

	def _viconReceive(self):
		# get header
		msg = self._receive(2*4)
		header = struct.unpack("<2L", msg)

		if header[0] not in [1,2]:
			# packet we are not set up to process
			return header

		msg = self._receive(1*4)
		length = struct.unpack("<1L", msg)

		if header[0] == 1:
			# info packet
			strs = []
			for i in xrange(length[0]):
				msg = self._receive(1*4)
				strlen = struct.unpack("<1L", msg)
				msg = self._receive(strlen[0])
				strs.append(msg)
			return strs
		elif header[0] == 2:
			# data packet
			msg = self._receive(length[0]*8)
			body = struct.unpack("<" + str(length[0]) + "d", msg)
			return body

	def _receive(self, msglen):
		msg = ''
		while len(msg) < msglen:
			chunk = self.sock.recv(msglen-len(msg))
			if chunk == '':
				raise RuntimeError("socket connection broken")
			msg = msg + chunk
		return msg

	def _processStream(self):
		while self._streaming:
			self.data = s._viconReceive()
			if self._verbose:
				print "  ".join([self._streamNames[i] for i in self._desiredStreams])
				for i in self._desiredStreams:
					print self.data[i], "  ",
		print


def connect_droneapi():
	"""
	Connects to droneapi. This is needed for getting data from the copter and connecting to MavProxy
	"""
	global api
	api = local_connect()
	global v
	v = api.get_vehicles()[0]
	print "API Connected..."

def connect_vicon():
	"""
	Connects to vicon. This is needed to scream vicon data.
	"""
	global s
	s = ViconStreamer()
	s.connect("Vicon", 800)
	streams = s.selectStreams(["Time", "t-", "a-"])
	s.startStreams(verbose=False)
	print "Vicon Connected..."

def connect_curses():
	"""
	Connects to curses. Need to make a simple GUI using curses.
	"""
	global use_curses
	global screen
	use_curses = True
	screen = curses.initscr()
	screen.refresh()

def disconnect_vicon():
	"""
	Properly closes vicon connection. Call this when finished using vicon.
	"""
	s.close()

def connect_all():
	"""
	Connects to droneapi, vicon, curses
	"""
	connect_droneapi()
	connect_vicon()
	connect_curses()

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
	if use_curses == True:
		if col == 0:
			screen.addstr(line, 0, string)
		if col == 1:
			screen.addstr(line, 40, string)

	screen.refresh()

def get_gains():
	"""
	Get the gain values from 'gains.txt'
	"""
	global alt_K_P
	global alt_K_I
	global yaw_K_P
	global yaw_K_I
	global roll_K_P
	global roll_K_I
	global roll_K_D
	global pitch_K_P
	global pitch_K_I
	global pitch_K_D

	#Get fill lines with the lines from the gain file
	gainfile = open('/home/tom/gains.txt', "r")
	lines = gainfile.readlines()
	gainfile.close()

	#Set the gain files from the values from the file
	alt_K_P = float(lines[1])
	alt_K_I = float(lines[3])
	yaw_K_P = float(lines[5])
	yaw_K_I = float(lines[7])
	roll_K_P = float(lines[9])
	roll_K_I = float(lines[11])
	roll_K_D = float(lines[13])
	pitch_K_P = float(lines[15])
	pitch_K_I = float(lines[17])
	pitch_K_D = float(lines[19])

def vicon_data():
	"""
	vicon_data()[0] = time
	vicon_data()[1] = x
	vicon_data()[2] = y
	vicon_data()[3] = z
	vicon_data()[4] = x rotation
	vicon_data()[5] = y rotation
	vicon_data()[6] = z rotation
	"""
	return s.getData()

def set_vicon_home():
	"""
	Set the home coordinate for the vicon data.
	"""
	home_x = vicon_data()[1]
	home_y = vicon_data()[2]
	home_z = vicon_data()[3]

def set_fence(min_x, max_x, min_y, max_y, min_z, max_z):
	"""
	Set the fence for the quadcopter to stay within. This currently just sets global fence variables.
	"""
	fence_x_min = mix_x
	fence_x_max = max_x
	fance_y_min = min_y
	fence_y_max = max_y
	fence_z_min = min_z
	fence_z_max = max_z

def arm():
	"""
	Arms the quadcopter
	"""
	print "Arming..."
	v.armed = True
	v.flush()

def disarm():
	"""
	Disarms the quadcopter.
	"""
	print "Disarming..."
	v.armed = False
	v.flush()

"""

Levels for actual quadcopter (Not SITL)

Throttle: 1117-1921 Mid: 1521
Pitch: 1111-1930 Mid: 1521
Yaw: 1111-1936 Mid: 1521
Roll: 1132-1920 Mid: 1520
Radio 5: Low: 968 High: 2072
Radio 6: Low: 968 High: 2073

"""

def rc_filter(rc_value):
	"""
	Filter for the RC values to filter out RC values out of RC range. Returns filtered RC value
	"""
	if rc_value > 2200.0:
		rc_value = 2200
	if rc_value < 500.0:
		rc_value = 500
	return rc_value

## Set RC Channels ##
def rc_roll(rc_value):
	rc_value = rc_filter(rc_value)
	if rc_check_dup('1', rc_value) == False:
		v.channel_override = { "1" : rc_value}
		v.flush()

def rc_pitch(rc_value):
	rc_value = rc_filter(rc_value)
	if rc_check_dup('2', rc_value) == False:
		v.channel_override = { "2" : rc_value}
		v.flush()

def rc_throttle(rc_value):
	rc_value = rc_filter(rc_value)
	if rc_check_dup('3', rc_value) == False:
		v.channel_override = { "3" : rc_value}
		v.flush()

def rc_yaw(rc_value):
	rc_value = rc_filter(rc_value)
	if rc_check_dup('4', rc_value) == False:
		v.channel_override = { "4" : rc_value}
		v.flush()

## Reset RC Channels ##
def rc_roll_reset():
	if rc_check_dup('1', -1) == False:
		v.channel_override = { "1" : -1}
		v.flush()

def rc_pitch_reset():
	if rc_check_dup('2', -1) == False:
		v.channel_override = { "2" : -1}
		v.flush()

def rc_throttle_reset():
	if rc_check_dup('3', -1) == False:
		v.channel_override = { "3" : -1}
		v.flush()

def rc_yaw_reset():
	if rc_check_dup('4', -1) == False:
		v.channel_override = { "4" : -1}
		v.flush()

def rc_six_reset():
	if rc_check_dup('6', -1) == False:
		v.channel_override = { "6" : -1}
		v.flush()

def rc_all_reset():
	rc_roll_reset()
	rc_pitch_reset()
	rc_throttle_reset()
	rc_yaw_reset()
	rc_six_reset()

def rc_check_dup(channel, value):
	"""
	Check for duplicate RC value. This is used to check to see if the RC value is already set to the vlue being passed in.
	"""
	if v.channel_readback[channel] == value:
		return True
	return False

def get_alt():
	"""
	Returns the altitude in mm in SITL
	"""
	return v.location_list[2]*1000

def get_lat():
	"""
	Returns the latitude of the copter in SITL
	"""
	return v.location_list[0]

def get_lon():
	"""
	Returns the longitude of the copter in SITL
	"""
	return v.location_list[1]

def get_roll():
	"""
	Returns the roll in radians
	"""
	return v.attitude_list[2]

def get_yaw_radians():
	"""
	Returns the current yaw in radians from -pi to pi
	Works for both SITL and Vicon
	For SITL it returns that yaw givn by the copter and for the Vicon system it returns the yaw given by the Vicon
	"""
	yaw = None
	if sitl == True:
		yaw = v.attitude_list[1]
	else:
		yaw = vicon_data()[6]
	return yaw

def get_yaw_degrees():
	"""
	Returns the current yaw in degrees from 0 to 360
	Works for SITL and Vicon
	For SITL it returns that yaw givn by the copter and for the Vicon system it returns the yaw given by the Vicon
	"""
	degrees = math.degrees(get_yaw_radians)*-1

	if degrees < 0.0:
		degrees += 360

	return degrees

def get_pitch():
	"""
	Returns the pitch of the copter in radians
	"""
	return v.attitude_list[0]

def get_position():
	"""
	Will return position in millimeters. (X,Y,Z)
	Use this for Vicon and SITL in the loop.
	"""
	position=[None]*3
	if sitl == True:
		position[0] = calc_sitl_distance_x()
		position[1] = calc_sitl_distance_y()
		position[2] = get_alt()
	else:
		position[0] = vicon_data()[1] - home_x
		position[1] = vicon_data()[2] - home_y
		psoition[2] = vicon_data()[3] - home_z

	return position

def get_distance():
	"""
	Returns the distance traveled from home in millimeters
	"""
	if sitl == True:
		distance = calc_sitl_distance(home_lat, home_lon, get_lat(), get_lon())
	else:
		distance = math.sqrt(get_position()[0]*getposition()[0] + get_position()[1]*get_position()[1])

	return distance

def set_sitl_home():
	"""
	Sets home for SITL
	"""
	global home_lat
	global home_lon
	global home_alt
	home_lat = get_lat()
	home_lon = get_lon()
	home_alt = get_alt()

def set_home():
	"""
	Sets the home for he quadcopter.
	Use this for Vicon and SITL
	"""
	if sitl == True:
		set_sitl_home()
	else:
		set_vicon_home()

def calc_sitl_distance(lat1, lon1, lat2, lon2):
	"""
	Calculates the distance from one point of lat-lon to another point of lat-lon.
	Uses the 'haversine' formula to calculte the distance between lat-lon points
	Returns distance in mm
	"""
	radius = 6371

	lat1_rad = math.radians(lat1)
	lat2_rad = math.radians(lat2)

	delta_lat_rad = math.radians(lat2-lat1)
	delta_lon_rad = math.radians(lon2-lon1)

	a = ( math.sin(delta_lat_rad/2) * math.sin(delta_lat_rad/2) ) + ( math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon_rad/2) * math.sin(delta_lon_rad/2) )

	c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

	#Returns a distance in meters
	return radius * c * 1000 * 1000


def calc_utm_distance(lat1, lon1, lat2, lon2):
	"""
	Returns distance in millimeters
	This is not currently used in any of the code. Left for now in case needed
	"""
	point1 = utm.from_latlon(lat1, lon1)
	point2 = utm.from_latlon(lat2, lon2)

	x = point2[1]-point1[1]
	y = point2[2]-point1[2]

	return 1000*math.sqrt(x*x+y*y)

#X axis is north/south (change in lat)
def calc_sitl_distance_x():
	"""
	Calculate x distance in SITL (lat)
	"""
	return calc_sitl_distance(home_lat, home_lon, home_lat, get_lon())

#Y axis is east/west (change in lon)
def calc_sitl_distance_y():
	"""
	Calculate y distance in SITL (lon)
	"""
	return calc_sitl_distance(home_lat, home_lon, get_lat(), home_lon)

def rc_go_to_alt(goal_alt):
	"""
	Will send copter based off of throttle to 'goal_alt'
	Can hold with about 0-10 millimeter precision but gets worse as time progresses
	"""
	global previous_time_alt
	global I_error_alt
	global error_alt
	global alt_K_P
	global alt_K_I

	#Calculate delta t and set previous time ot current time
	current_time = ((time.clock()-timer)*10)
	delta_t = current_time - previous_time_alt
	previous_time_alt = current_time

	#Get error
	error_alt = goal_alt - get_alt()

	#Get error I
	I_error_alt = I_error_alt + error_alt*delta_t

	#Print alt data
	curses_print("Throttle RC Level: " + str(v.channel_readback['3']), 6, 1)
	curses_print("Error: " + str(error_alt), 7, 1)
	curses_print("Altitude:" + str(get_alt()), 8, 1)
	curses_print("Throttle RC = 1370 + " + str(error_alt*alt_K_P) + " + " + str(I_error_alt*alt_K_I), 19, 0)

	#Send RC value
	rc_throttle(1370+error_alt*alt_K_P+I_error_alt*alt_K_I)

	return error_alt

def rc_go_to_heading(goal_heading):
	"""
	Sends quad to given yaw
	Imput is in radians from -pi to pi
	"""

	global previous_time_yaw
	global I_error_yaw
	global eror_yaw
	global yaw_K_P
	global yaw_K_I

	#Get rid of bad inputs
	if goal_heading > 3.1415926 or goal_heading < -3.14159:
		return 0

	#Calculate delta t and set previous time ot current time
	current_time = ((time.clock()-timer)*10)
	delta_t = current_time - previous_time_yaw
	previous_time_yaw = current_time

	#Get error
	error_yaw = goal_heading - get_yaw_radians()

	#Get error I
	I_error_yaw = I_error_yaw + error_yaw*delta_t

	#Print yaw data
	curses_print("Yaw RC Level: " + str(v.channel_readback['4']), 6, 0)
	curses_print("Error: " + str(error_yaw), 7, 0)
	curses_print("Heading Radians: " + str(get_yaw_radians()), 8, 0)
	curses_print("Heading Degrees: " + str(get_yaw_degrees()), 9, 0)
	curses_print("Yaw RC      = 1500 + " + str(error_yaw*yaw_K_P) + " + " + str(I_error_yaw*yaw_K_I), 20, 0)

	#Send RC value
	rc_yaw(1500+error_yaw*yaw_K_P+I_error_yaw*yaw_K_I)

	return error_alt


def rc_go_to_xy(goal_x, goal_y):
	"""
	Sends quad copter to given x-y point
	"""

	global previous_time_xy
	global previous_error_pitch
	global previous_error_roll
	global D_error_roll
	global D_error_pitch
	global I_error_roll
	global I_error_pitch
	global sum_lat
	global sum_lon
	global count_lat_lon
	global error_roll
	global error_pitch
	global x_current
	global y_current
	global pitch_K_P
	global pitch_K_I
	global pitch_K_D
	global roll_K_P
	global roll_K_I
	global roll_K_D
	global error_x
	global error_y

	#Average out lat and lon (May not be needed. Need to text this)
	sum_lat += get_lat()
	sum_lon += get_lon()
	count_lat_lon += 1
	average_lat = 0.0
	average_lon = 0.0
	if count_lat_lon != 1:
		return
	average_lat = sum_lat / 1.0
	average_lon = sum_lon / 1.0
	sum_lat = 0
	sum_lon = 0
	count_lat_lon = 0

	#Get current heading for shifting axis
	heading = get_yaw_degrees()

	#Calculate current position (Need to find which one works best)
	#x_current = calc_sitl_distance_x()
	#y_current = calc_sitl_distance_y()
	x_current = calc_sitl_distance(home_lat, home_lon, home_lat, average_lon)
	y_current = calc_sitl_distance(home_lat, home_lon, average_lat, home_lon)
	#x_current = calc_utm_distance(home_lat, home_lon, home_lat, get_lon())
	#y_current = calc_utm_distance(home_lat, home_lon, get_lat(), home_lon)

	#Assign distance with appropriate sign
	if get_lat() < home_lat:
		y_current *= -1
	if get_lon() < home_lon:
		x_current *= -1

	#Calculate the error in the x-y(lat/lon) axis
	error_x = goal_x - x_current * 1.0
	error_y = goal_y - y_current * 1.0

	#Make error not zero so you don't get a divid by zero error (Need to tes to see if needed)
	if error_x == 0:
		error_x += .000000000001

	if error_y == 0:
		error_y += .000000000001

	#Total error from current point to goal point
	total_error = math.sqrt(error_x*error_x+error_y*error_y)

	#Angle on x-y(lat/lon) axis to point
	waypoint_angle = math.degrees(math.atan(error_y/error_x))

	#Put angle in the correct quadrant
	if error_x < 0 and error_y < 0:
		waypoint_angle += 180
	if error_x < 0 and error_y > 0:
		waypoint_angle += 180
	if error_x > 0 and error_y < 0:
		waypoint_angle += 360

	#Calculate the offset of the vehicle from the x-y (lat-lon) axis
	vehicle_angle = 90 - (waypoint_angle + heading)

	#Calculate the error for the roll and pitch
	error_roll = total_error * math.sin(math.radians(vehicle_angle))
	error_pitch = total_error * math.cos(math.radians(vehicle_angle))*-1

	#Print functions for curses
	curses_print("Pitch RC Level: " + str(v.channel_readback['2']), 11, 0)
	curses_print("Roll RC Level: " + str(v.channel_readback['1']), 11, 1)
	curses_print("Pitch: " + str(get_pitch()), 12, 0)
	curses_print("Roll: " + str(get_roll()), 12, 1)
	curses_print("X Error: " + str(round(error_x)), 15, 0)
	curses_print("Y Error: " + str(round(error_y)), 15, 1)
	curses_print("Roll Error: " + str(round(error_roll)), 13, 1)
	curses_print("Pitch Error: " + str(round(error_pitch)), 13, 0)
	curses_print("Total Error: " + str(round(total_error)), 16, 0)
	#curses_print("Lat, Lon: " + str(home_lat) + ", " + str(home_lon), 18)
	#curses_print("Lat, Lon: " + str(average_lat) + ", " + str(average_lon), 19)


	#Calculate delta-t for integration
	current_time = ((time.clock()-timer)*10)
	delta_t = current_time - previous_time_xy
	previous_time_xy = current_time

	#Calculate the I error for roll and pitch
	I_error_roll = I_error_roll + error_roll*delta_t
	I_error_pitch = I_error_pitch + error_pitch*delta_t

	#Calculate the D error for roll and pitch
	D_error_roll = (error_roll-previous_error_roll)/delta_t
	D_error_pitch = (error_pitch-previous_error_pitch)/delta_t

	previous_error_pitch = error_pitch
	previous_error_roll = error_roll

	curses_print("Pitch RC    = 1505 + " + str(error_pitch*pitch_K_P) + " + " + str(I_error_pitch*pitch_K_I) + " + " + str(D_error_pitch*pitch_K_D), 21, 0)
	curses_print("Roll  RC    = 1505 + " + str(error_roll*roll_K_P) + " + " + str(I_error_roll*roll_K_I) + " + " + str(D_error_roll*roll_K_D), 22, 0)

	#Send RC values
	rc_pitch( 1505 + (error_pitch*pitch_K_P) + (I_error_pitch*pitch_K_I) + (D_error_pitch*pitch_K_D) )
	rc_roll(  1505 + (error_roll*roll_K_P) + (I_error_roll*roll_K_I) + (D_error_roll*roll_K_D) )



##### Vidro Test Code #####

connect_droneapi()
connect_curses()

#Set home
time.sleep(1)
set_home()
time.sleep(1)

#Start the time clock
timer = time.clock()


#Main program loop
while v.channel_readback['6'] < 1100:


	#Set the gains from the gain file
	get_gains()


	#Setting goal
	rc_go_to_alt(10000)
	rc_go_to_heading(0)
	rc_go_to_xy(0, 0)


	#Add values to arrays for plotting
	plot_error_yaw.append(error_yaw)
	plot_error_yaw_I.append(I_error_yaw)
	plot_time_yaw.append(previous_time_yaw)

	plot_error_throttle.append(error_alt)
	plot_error_throttle_I.append(I_error_alt)
	plot_time_throttle.append(previous_time_alt)

	plot_error_pitch.append(error_pitch)
	plot_error_pitch_I.append(I_error_pitch)
	plot_time_pitch.append(previous_time_xy)
	plot_error_pitch_D.append(D_error_pitch)
	plot_rc_pitch.append(v.channel_readback['2'])

	plot_error_roll.append(error_roll)
	plot_error_roll_I.append(I_error_roll)
	plot_time_roll.append(previous_time_xy)
	plot_rc_roll.append(v.channel_readback['1'])
	plot_error_roll_D.append(D_error_roll)

	plot_x_current.append(x_current)
	plot_y_current.append(y_current)


	#Print out of time
	curses_print("Time: " + str((time.clock()-timer)*10),0,0)

	#Formatting for PID
	curses_print("              Base        P                I                 D", 18, 0)

	#
	curses_print("             X              Y              Z              YAW", 2, 0)
	curses_print("Position = " + str(get_position()[0]) + " " + str(get_position()[1]) + " " + str(get_position()[2]) + " " + str(get_yaw_radians()), 3, 0)
	curses_print("Error    = " + str(error_x) + " " + str(error_y) + " " + str(error_alt) + " " + str(error_yaw), 4, 0)

	#Sleep
	time.sleep(.1)


disarm()


#Plots
"""
plot.figure(1)
plot.xlabel("Time(sec)")
plot.ylabel("Error(rads)")
plot.title("Yaw")
plot.plot(plot_time_yaw,plot_error_yaw)
plot.figure(2)
plot.xlabel("Time(sec)")
plot.ylabel("Error(mm)")
plot.title("Throttle")
plot.plot(plot_time_throttle,plot_error_throttle)
"""
plot.figure(3)
plot.xlabel("Time(sec)")
plot.ylabel("Error(mm) | RC Value")
plot.title("Pitch Error PD | RC Value")
plot.plot(plot_time_pitch,plot_error_pitch)
plot.plot(plot_time_pitch,plot_rc_pitch)
plot.plot(plot_time_pitch,plot_error_pitch_D)
plot.figure(4)
plot.xlabel("Time(sec)")
plot.ylabel("Error(mm) | RC Value")
plot.title("Roll Error PD | RC Value")
plot.plot(plot_time_roll,plot_error_roll)
plot.plot(plot_time_roll,plot_rc_roll)
plot.plot(plot_time_roll,plot_error_roll_D)
"""
plot.figure(5)
plot.xlabel("Time(sec)")
plot.ylabel("Error(rads)")
plot.title("Yaw")
plot.plot(plot_time_yaw,plot_error_yaw)
plot.plot(plot_time_yaw,plot_error_yaw_I)
plot.figure(6)
plot.xlabel("Time(sec)")
plot.ylabel("Error(mm)")
plot.title("Throttle")
plot.plot(plot_time_throttle,plot_error_throttle)
plot.plot(plot_time_throttle,plot_error_throttle_I)
"""
plot.figure(7)
plot.xlabel("Time(sec)")
plot.ylabel("Error(mm)")
plot.title("Pitch Error PI")
plot.plot(plot_time_pitch,plot_error_pitch)
plot.plot(plot_time_pitch,plot_error_pitch_I)
plot.figure(8)
plot.xlabel("Time(sec)")
plot.ylabel("Error(mm)")
plot.title("Roll Error PI")
plot.plot(plot_time_roll,plot_error_roll)
plot.plot(plot_time_roll,plot_error_roll_I)
plot.figure(9)
plot.xlabel("x Location(mm)")
plot.ylabel("y Location(mm)")
plot.title("Location")
plot.plot(plot_x_current, plot_y_current)
plot.show()
