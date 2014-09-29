from droneapi.lib import VehicleMode
from pymavlink import mavutil
import sys, math, time
import socket, struct, threading
import curses
import utm
import matplotlib.pyplot as plot

sitl = True

use_curses = False

home_x = 0
home_y = 0
home_z = 0

home_lat = 0
home_lon = 0
home_alt = 0

fence_x_min = 0
fence_x_max = 0
fance_y_min = 0
fence_y_max = 0
fence_z_min = 0
fence_z_max = 0

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

sum_lat = 0
sum_lon = 0
count_lat_lon = 0

x_current = 0
y_current = 0
	
#ViconStreamer code from: Cameron Finucane <cpf37@cornell.edu>
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
	global api
	api = local_connect()
	global v
	v = api.get_vehicles()[0]
	print "API Connected..."
	
def connect_vicon():
	global s
	s = ViconStreamer()
	s.connect("Vicon", 800)
	streams = s.selectStreams(["Time", "t-", "a-"])
	s.startStreams(verbose=False)
	print "Vicon Connected..."
	
def connect_curses():
	global use_curses
	global screen
	use_curses = True
	screen = curses.initscr()
	screen.refresh()
	
def disconnect_vicon():
	s.close()

def connect_all():
	connect_droneapi()
	connect_vicon()
	connect_curses()
	
def curses_print(string, line):
	if use_curses == True:
		screen.addstr(line, 0, string)
			
	screen.refresh()
	
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
	home_x = vicon_data()[1]
	home_y = vicon_data()[2]
	home_z = vicon_data()[3]

def set_fence(min_x, max_x, min_y, max_y, min_z, max_z):
	fence_x_min = mix_x
	fence_x_max = max_x
	fance_y_min = min_y
	fence_y_max = max_y
	fence_z_min = min_z
	fence_z_max = max_z
	
def arm():
	print "Arming..."
	v.armed = True
	v.flush()

def disarm():
	print "Disarming..."
	v.armed = False
	v.flush()

"""
Throttle: 1117-1921 Mid: 1521
Pitch: 1111-1930 Mid: 1521
Yaw: 1111-1936 Mid: 1521
Roll: 1132-1920 Mid: 1520
Radio 5: Low: 968 High: 2072
Radio 6: Low: 968 High: 2073
"""
def rc_filter(rc_value):
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
	
def rc_print():
	print "Current overrides are:", v.channel_override
	print "RC readback:", v.channel_readback
	
def rc_check_dup(channel, value):
	if v.channel_readback[channel] == value:
		return True
	return False

def get_alt():
	return v.location_list[2]*1000
	
def get_lat():
	return v.location_list[0]

def get_lon():
	return v.location_list[1]
	
def get_roll():
	return v.attitude_list[2]
	
def get_yaw_radians():
	"""
	Returns the current yaw in radians from -pi to pi
	"""
	return v.attitude_list[1]
	
def get_yaw_degrees():
	"""
	Returns the current yaw in degrees from 0 to 360
	"""
	degrees = math.degrees(v.attitude_list[1])*-1

	if degrees < 0.0:
		degrees += 360
		
	return degrees
	
def get_pitch():
	return v.attitude_list[0]
	
def get_position():
	"""
	Will return position in millimeters. (X,Y,Z)
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
	global home_lat
	global home_lon
	global home_alt
	home_lat = get_lat()
	home_lon = get_lon()
	home_alt = get_alt()
	
def set_home():
	if sitl == True:
		set_sitl_home()
	else:
		set_vicon_home()
	
def calc_sitl_distance(lat1, lon1, lat2, lon2):
	#Use the 'haversine' formula to calculte the distance between lat-lon points
	
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
	"""
	point1 = utm.from_latlon(lat1, lon1)
	point2 = utm.from_latlon(lat2, lon2)
	
	x = point2[1]-point1[1]
	y = point2[2]-point1[2]
	
	return 1000*math.sqrt(x*x+y*y)
	
#X axis is north/south (change in lat)
def calc_sitl_distance_x():
	return calc_sitl_distance(home_lat, home_lon, get_lat(), home_lon)
	
#Y axis is east/west (change in lon)
def calc_sitl_distance_y():
	return calc_sitl_distance(home_lat, home_lon, home_lat, get_lon())

def rc_go_to_alt(goal_alt):
	"""
	Will send copter based off of throttle to 'goal_alt'
	Can hold with about 0-10 millimeter precision but gets worse as time progresses
	"""
	global previous_time_alt
	global I_error_alt
	global error_alt
	
	current_time = ((time.clock()-timer)*10)
	delta_t = current_time - previous_time_alt
	previous_time_alt = current_time
	
	error_alt = goal_alt - get_alt()
	
	I_error_alt = I_error_alt + error_alt*delta_t
	
	curses_print("Throttle RC Level: " + str(v.channel_readback['3']), 2)
	curses_print("Error: " + str(error_alt), 3)
	curses_print("Altitude:" + str(get_alt()), 4)
	rc_throttle(1370+error_alt*.0050+I_error_alt*.00003)
	screen.refresh()
	return error_alt	
	
def rc_go_to_heading(goal_heading):
	"""
	Imput is in radians from -pi to pi
	"""
	global previous_time_yaw
	global I_error_yaw
	global eror_yaw
	
	if goal_heading > 3.1415926 or goal_heading < -3.14159:
		return 0
	
	current_time = ((time.clock()-timer)*10)
	delta_t = current_time - previous_time_yaw
	previous_time_yaw = current_time
	
	error_yaw = goal_heading - get_yaw_radians()
	
	I_error_yaw = I_error_yaw + error_yaw*delta_t
	
	curses_print("Yaw RC Level: " + str(v.channel_readback['4']), 6)
	curses_print("Error: " + str(error_yaw), 7)
	curses_print("Heading Radians: " + str(get_yaw_radians()), 8)
	curses_print("Heading Degrees: " + str(get_yaw_degrees()), 9)
	rc_yaw(1500+error_yaw*130+I_error_yaw*10)
	return error_alt

	
def rc_go_to_xy(goal_x, goal_y):
	
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
	x_error = goal_x - x_current * 1.0
	y_error = goal_y - y_current * 1.0
	
	#Make error not zero so you don't get a divid by zero error (Need to tes to see if needed)
	if x_error == 0:
		x_error += .000000000001
		
	if y_error == 0:
		y_error += .000000000001
		
	#Total error from current point to goal point
	total_error = math.sqrt(x_error*x_error+y_error*y_error)
	
	#Angle on x-y(lat/lon) axis to point
	waypoint_angle = math.degrees(math.atan(y_error/x_error))
	
	#Put angle in the correct quadrant
	if x_error < 0 and y_error < 0:
		waypoint_angle += 180
	if x_error < 0 and y_error > 0:
		waypoint_angle += 180
	if x_error > 0 and y_error < 0:
		waypoint_angle += 360
	
	#Calculate the offset of the vehicle from the x-y (lat-lon) axis
	vehicle_angle = 90 - (waypoint_angle + heading)
	
	#Calculate the error for the roll and pitch
	error_roll = total_error * math.sin(math.radians(vehicle_angle))
	error_pitch = total_error * math.cos(math.radians(vehicle_angle))*-1
	
	curses_print("Pitch RC Level: " + str(v.channel_readback['2']), 11)
	curses_print("Roll RC Level: " + str(v.channel_readback['1']), 12)
	curses_print("X Error: " + str(round(x_error)), 13)
	curses_print("Y Error: " + str(round(y_error)), 14)
	curses_print("Total Error: " + str(round(total_error)), 15)
	curses_print("Roll Error: " + str(round(error_roll)), 16)
	curses_print("Pitch Error: " + str(round(error_pitch)), 17)
	curses_print("Lat, Lon: " + str(home_lat) + ", " + str(home_lon), 18)
	curses_print("Lat, Lon: " + str(average_lat) + ", " + str(average_lon), 19)
	curses_print("Pitch: " + str(get_pitch()), 20)
	curses_print("Roll: " + str(get_roll()), 21)
	
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
	
	#Send RC values
	rc_pitch( 1537 + (error_pitch*.09) + (I_error_pitch*.00001) + (D_error_pitch*.05) )
	rc_roll(  1537 + (error_roll* .09)  + (I_error_roll*.00001) + (D_error_roll*.05) )
	
#Test

connect_droneapi()
connect_curses()


time.sleep(1)
set_home()
time.sleep(1)
timer = time.clock()

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

while v.channel_readback['6'] < 1100:
	
	rc_go_to_alt(10000)
	
	rc_go_to_heading(0)
	
	rc_go_to_xy(0, 0)
	
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
	
	curses_print("Time: " + str((time.clock()-timer)*10),0)
	time.sleep(.1)
	
disarm()

plot.figure(1)
plot.xlabel("Time(sec)")
plot.ylabel("Error(rads)")
plot.title("Yaw Error")
plot.plot(plot_time_yaw,plot_error_yaw)
plot.figure(2)
plot.xlabel("Time(sec)")
plot.ylabel("Error(mm)")
plot.title("Altitude Error")
plot.plot(plot_time_throttle,plot_error_throttle)

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


plot.figure(7)
plot.xlabel("Time(sec)")
plot.ylabel("Error(mm)")
plot.title("Pitch Error P")
plot.plot(plot_time_pitch,plot_error_pitch)
#plot.plot(plot_time_pitch,plot_error_pitch_I)
plot.figure(8)
plot.xlabel("Time(sec)")
plot.ylabel("Error(mm)")
plot.title("Roll Error P")
plot.plot(plot_time_roll,plot_error_roll)
#plot.plot(plot_time_roll,plot_error_roll_I)

plot.figure(9)
plot.xlabel("x Location(mm)")
plot.ylabel("y Location(mm)")
plot.title("Location")
plot.plot(plot_x_current, plot_y_current)
plot.show()
