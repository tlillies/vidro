from droneapi.lib import VehicleMode
from pymavlink import mavutil
import sys, math, time
import socket, struct, threading

global home_x
global home_y
global home_z

home_x = 0
home_y = 0
home_z = 0

global home_lat
global home_lon
global home_alt

home_lat = 0
home_lon = 0
home_alt = 0

global fence_x_min
global fence_x_max
global fance_y_min
global fence_y_max
global fence_z_min
global fence_z_max

fence_x_min = 0
fence_x_max = 0
fance_y_min = 0
fence_y_max = 0
fence_z_min = 0
fence_z_max = 0
	
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
	return s

def connect_all():
	connect_droneapi()
	connect_vicon()
	
def vicon_data(s):
	#vicon_data()[0] = time
	#vicon_data()[1] = x
	#vicon_data()[2] = y
	#vicon_data()[3] = z
	#vicon_data()[4] = x rotation
	#vicon_data()[5] = y rotation
	#vicon_data()[6] = z rotation
	return s.getData()
	"""
def set_home(x=vicon_data()[1], y=vicon_data()[2], z=vicon_data()[3]):
	home_x = x
	home_y = y
	home_z = z
"""
def set_fence(min_x, max_x, min_y, max_y, min_z, max_z):
	fence_x_min = mix_x
	fence_x_max = max_x
	fance_y_min = min_y
	fence_y_max = max_y
	fence_z_min = min_z
	fence_z_max = max_z
	

#Message Factoy Documentation
"""
def message_factory(self):
        
        Returns an object that can be used to create 'raw' mavlink messages that are appropriate for this vehicle.
        These message types are defined in the central Mavlink github repository.  For example, a Pixhawk understands
        the following messages: (from https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/pixhawk.xml).##

          <message id="153" name="IMAGE_TRIGGER_CONTROL">
               <field type="uint8_t" name="enable">0 to disable, 1 to enable</field>
          </message>

        The name of the factory method will always be the lower case version of the message name with _encode appended.
        Each field in the xml message definition must be listed as arguments to this factory method.  So for this example
        message, the call would be:

        msg = vehicle.message_factory.image_trigger_control_encode(True)
        vehicle.send_mavlink(msg)
        
        return self.__module.master.mav
  
        """
        
        
#Other Example
"""
#Test custom commands
#Note: For mavlink messages that include a target_system & target_component, those values
#can just be filled with zero.  The API will take care of using the correct values
#For instance, from the xml for command_long:
#                Send a command with up to seven parameters to the MAV
#
#                target_system             : System which should execute the command (uint8_t)
#                target_component          : Component which should execute the command, 0 for all components (uint8_t)
#                command                   : Command ID, as defined by MAV_CMD enum. (uint16_t)
#                confirmation              : 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command) (uint8_t)
#                param1                    : Parameter 1, as defined by MAV_CMD enum. (float)
#                param2                    : Parameter 2, as defined by MAV_CMD enum. (float)
#                param3                    : Parameter 3, as defined by MAV_CMD enum. (float)
#                param4                    : Parameter 4, as defined by MAV_CMD enum. (float)
#                param5                    : Parameter 5, as defined by MAV_CMD enum. (float)
#                param6                    : Parameter 6, as defined by MAV_CMD enum. (float)
#                param7                    : Parameter 7, as defined by MAV_CMD enum. (float)
msg = v.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, 0, 0, 0, 0, 1, 0, 0)
print "Created msg: %s" % msg
v.send_mavlink(msg)
"""


def condition_yaw(yaw):
	msg = v.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, yaw, 0, 0, 0, 1, 0, 0)
	print "Created msg: %s" % msg
	v.send_mavlink(msg)
	
#115
#MAV_CMD_CONDITION_YAW	Reach a certain target angle.

#Mission Param #1	target angle: [0-360], 0 is north
#Mission Param #2	speed during yaw change:[deg per second]
#Mission Param #3	direction: negative: counter clockwise, positive: clockwise [-1,1]
#Mission Param #4	relative offset or absolute angle: [ 1,0]
#Mission Param #5	Empty
#Mission Param #6	Empty
#Mission Param #7	


	
def condition_speed(speed):
	msg = v.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_CONDITION_DO_CHANGE_SPEED, 0, 1, speed, 0, 0, 1, 0, 0)
	print "Created msg: %s" % msg
	v.send_mavlink(msg)
	
#178	
#MAV_CMD_DO_CHANGE_SPEED	Change speed and/or throttle set points.

#Mission Param #1	Speed type (0=Airspeed, 1=Ground Speed)
#Mission Param #2	Speed (m/s, -1 indicates no change)
#Mission Param #3	Throttle ( Percent, -1 indicates no change)
#Mission Param #4	Empty
#Mission Param #5	Empty
#Mission Param #6	Empty
#Mission Param #7	Empty
	
	
	
	
def condition_alt(alt):
	msg = v.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT, 0, alt, 0, 0, 0, 1, 0, 0)
	print "Created msg: %s" % msg
	v.send_mavlink(msg)
	
#113
#MAV_CMD_CONDITION_CHANGE_ALT	Ascend/descend at rate. Delay mission state machine until desired altitude reached.

#Mission Param #1	Descent / Ascend rate (m/s)
#Mission Param #2	Empty
#Mission Param #3	Empty
#Mission Param #4	Empty
#Mission Param #5	Empty
#Mission Param #6	Empty
#Mission Param #7	Finish Altitude





	
	
	
def manual_control(x,y,z,r,buttons):
	msg = v.message_factory.manual_control_encode(0, x, y, z, r, buttons)
	print "Created msg: %s" % msg
	v.send_mavlink(msg)
	
#MANUAL_CONTROL ( #69 )
#This message provides an API for manually controlling the vehicle using standard joystick axes nomenclature, along with a joystick-like input device. Unused axes can be disabled an buttons are also transmit as boolean values of their

#target		uint8_t		The system to be controlled.
#x			int16_t		X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle.
#y			int16_t		Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle.
#z			int16_t		Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on a joystick and the thrust of a vehicle.
#r			int16_t		R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise being -1000, and the yaw of a vehicle.
#buttons	uint16_t	A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 1.
	


	
def manual_setpoint(roll,pitch,yaw,thrust):
	msg = v.message_factory.manual_setpoint_encode(0, roll, pitch, yaw, thrust, 0, 0)
	print "Created msg: %s" % msg
	v.send_mavlink(msg)
	
#MANUAL_SETPOINT ( #81 )
#Setpoint in roll, pitch, yaw and thrust from the operator

#time_boot_ms			uint32_t	Timestamp in milliseconds since system boot
#roll					float		Desired roll rate in radians per second
#pitch					float		Desired pitch rate in radians per second
#yaw					float		Desired yaw rate in radians per second
#thrust					float		Collective thrust, normalized to 0 .. 1
#mode_switch			uint8_t		Flight mode switch position, 0.. 255
#manual_override_switch	uint8_t		Override mode switch position, 0.. 255




def set_attitude_target(roll_rate, pitch_rate,):
	msg = v.message_factory.set_attitude_target_encode(0, roll, pitch, yaw, thrust, 0, 0)
	print "Created msg: %s" % msg
	v.send_mavlink(msg)
	
#SET_ATTITUDE_TARGET ( #82 )
#Set the vehicle attitude and body angular rates.

#time_boot_ms		uint32_t	Timestamp in milliseconds since system boot
#target_system		uint8_t		System ID
#target_component	uint8_t		Component ID
#type_mask			uint8_t		Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude
#q					float[4]	Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
#body_roll_rate		float		Body roll rate in radians per second
#body_pitch_rate	float		Body roll rate in radians per second
#body_yaw_rate		float		Body roll rate in radians per second
#thrust				float		Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)





def set_attitude_target():
	print "Created msg: %s" % msg
	v.send_mavlink(msg)

#SET_ATTITUDE_TARGET ( #83 )
#Set the vehicle attitude and body angular rates.

#time_boot_ms		uint32_t	Timestamp in milliseconds since system boot
#type_mask			uint8_t		Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude
#q					float[4]	Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
#body_roll_rate		float		Body roll rate in radians per second
#body_pitch_rate	float		Body roll rate in radians per second
#body_yaw_rate		float		Body roll rate in radians per second
#thrust				float		Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)





def vicon_position_estimate():
	print "Created msg: %s" % msg
	v.send_mavlink(msg)

#VICON_POSITION_ESTIMATE ( #104 )
#usec	uint64_t	Timestamp (microseconds, synced to UNIX time or since system boot)
#x		float		Global X position
#y		float		Global Y position
#z		float		Global Z position
#roll	float		Roll angle in rad
#pitch	float		Pitch angle in rad
#yaw	float		Yaw angle in rad


def send_nav_velocity(velocity_x, velocity_y, velocity_z):
	# create the CONDITION_YAW command
    msg = v.message_factory.mission_item_encode(0, 0,  # target system, target component
													0,     # sequence
                                                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # frame
                                                    91, # command id, replace with mavutil.mavlink.MAV_CMD_NAV_VELOCITY
                                                    2, # current - set to 2 to make it a guided command
                                                    0, # auto continue
                                                    0,                         # frame (unused)
                                                    0, 0, 0,                   # params 1 ~ 4 (unused)
                                                    velocity_x, velocity_y, velocity_z) # params 5 ~ 7
	# send command to vehicle
    v.send_mavlink(msg)
    v.flush()


def arm():
	print "Arming..."
	v.armed = True
	v.flush()

def disarm():
	print "Disarming..."
	v.armed = False
	v.flush()

## Set RC Channels ##
def rc_roll(rc_value):
	if rc_check_dup('1', rc_value) == False:
		v.channel_override = { "1" : rc_value}
		v.flush()

def rc_pitch(rc_value):
	if rc_check_dup('2', rc_value) == False:
		v.channel_override = { "2" : rc_value}
		v.flush()

def rc_throttle(rc_value):
	if rc_check_dup('3', rc_value) == False:
		v.channel_override = { "3" : rc_value}
		v.flush()
	
def rc_yaw(rc_value):
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
	return v.location_list[2]
	
def get_lat():
	return v.location_list[0]

def get_lon():
	return v.location_list[1]
	
def get_roll():
	return v.attitude_list[2]
	
def get_yaw():
	return v.attitude_list[1]
	
def get_pitch():
	return v.attitude_list[0]

def set_sitl_home():
	global home_lat
	global home_lon
	global home_alt
	home_lat = get_lat()
	home_lon = get_lon()
	home_alt = get_alt()
	print home_lat
	
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
	return radius * c * 1000
	
#X axis is north/south (change in lat)
def calc_sitl_distance_x():
	return calc_sitl_distance(home_lat, home_lon, get_lat(), home_lon)
	
#Y axis is east/west (change in lon)
def calc_sitl_distance_y():
	return calc_sitl_distance(home_lat, home_lon, home_lat, get_lon())
	

def rc_go_to_alt(goal_alt):
	"""
	Will send copter based off of throttle to 'goal_alt'
	Can get there with .02 meter precision
	"""
	diff = goal_alt - get_alt()
	print "Throttle RC Level: " + str(v.channel_readback['3'])
	rc_throttle(1370+diff*7.5)
	time.sleep(.1)
	print "Off by: " + str(diff)
	print "Altitude:" + str(get_alt())
	
def rc_go_to_heading(goal_heading):
	diff = goal_heading - get_yaw()
	print "Yaw RC Level: " + str(v.channel_readback['4'])
	rc_yaw(1500+diff*7.5)
	time.sleep(.1)
	print "Off by: " + str(diff)
	print "Heading: "
	
#Test
connect_droneapi()

set_sitl_home()

time.sleep(2)

clock = time.clock()
while v.channel_readback['6'] == 1000:
	
	rc_go_to_alt(20)
	rc_go_to_heading(180)
	print "T:" + str(time.clock() - clock)
	print "X: " + str(calc_sitl_distance_x())
	print "Y: " + str(calc_sitl_distance_y())
	print "D: " + str(calc_sitl_distance(home_lat, home_lon, get_lat(), get_lon()))

disarm()











###Ignore###
#condition_alt(10)
#condition_speed(70)
#condition_yaw(200)
#manual_setpoint(1,1,1,1)

#test = connect_vicon()
#print str(vicon_data(test)[1])
#v.channel_override = { "3" : 1500}
#condition_alt(10)

#print "Current overrides are:", v.channel_override



