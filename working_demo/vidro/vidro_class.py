"""
Class for connecting to vicon and the APM
This is currently the main file used
"""
from pymavlink import mavutil
import sys, math, time
import socket, struct, threading
import logging
import vicon
import signal

class Vidro:

	def __init__(self, sitl, vicon_num):
		self.sitl = sitl
		if self.sitl == True:
			self.baud = 115200
			self.device = "127.0.0.1:14551"
		else:
			#self.baud = 57600
			#self.device = '/dev/ttyUSB0'
			self.baud = 115200
			self.device = '/dev/ttyACM0'

		#Home x,y,x position
		self.home_x = 0
		self.home_y = 0
		self.home_z = 0

		#Home lat,lon, and alt for sitl
		self.home_lat = 0
		self.home_lon = 0
		self.home_alt = 0

		#Last updated lat,lon, and alt for sitl
		self.current_lat = None
		self.current_lon = None
		self.current_alt = None

		self.ground_alt = 0

		#Last updated position
		self.current_x = None
		self.current_y = None
		self.current_z = None

		#Last updated rc channel's values'
		self.current_rc_channels = [None] * 6

		#Last updated rc overrides
		self.current_rc_overrides = [0] * 6

		#Last updated attitude
		self.current_pitch = None
		self.current_yaw = None
		self.current_roll = None

		#Fence for safety (Not implemented yet)
		self.fence_x_min = None
		self.fence_x_max = None
		self.fence_y_min = None
		self.fence_y_max = None
		self.fence_z_min = None
		self.fence_z_max = None

		self.clock = time.time()

		#Flag for vicon error. May not be needed
		self.vicon_error = False

		#The number of vicon objects that are being streamed. Can currently handle only two
		self.num_vicon_objs = vicon_num

		#Start of a log
		logging.basicConfig(filename='vidro.log', level=logging.DEBUG)

	def connect_mavlink(self):
		"""
		Initialize connection to pixhawk and make sure to get first heartbeat message
		"""
		#Initialize connection
		self.master = mavutil.mavlink_connection(self.device, self.baud)
		print "Attempting to get HEARTBEAT message from APM..."

		#Request heartbeat from APM
		msg = self.master.recv_match(type='HEARTBEAT', blocking=True)
		print("Heartbeat from APM (system %u component %u)" % (self.master.target_system, self.master.target_system))

		#The max rate (the second to last argument in the line below) is 25 Hz. You must change the firmware to get a fast rate than that.
		#It may be possible to get up to 500 Hz??
		#This may be useful later down the road to decrease latency
		#It also may be helpful to only stream needed data instead of all data
		if self.sitl == True:
			self.master.mav.request_data_stream_send(self.master.target_system, self.master.target_component, 0, 25, 1)
		else:
			self.master.mav.request_data_stream_send(self.master.target_system, self.master.target_component, 0, 1, 0) #All
			self.master.mav.request_data_stream_send(self.master.target_system, self.master.target_component, 3, 25, 1) #RC channels
			#self.master.mav.request_data_stream_send(self.master.target_system, self.master.target_component, 6, 25, 1) #Position

		#Get intial values from the APM
		print "Getting inital values from APM..."
		if self.sitl == False:
			while (self.current_rc_channels[0] == None):
				self.update_mavlink()
			print("Got RC channels")
		else:
			while (self.current_rc_channels[0] == None) or (self.current_alt == None) or (self.current_yaw == None):
				self.update_mavlink()
			print("Got RC channels, global position, and attitude")

		#Set contants for SITl
		if self.sitl == True:
			self.ground_alt = self.current_alt
			self.current_alt = 0
			print("Successfully set ground altitude")
			self.home_lat = self.current_lat
			self.home_lon = self.current_lon
			print("Successfully set home latitude and longitude")

	def update_mavlink(self):
		"""
		Function for getting the general mavlink message and changing class variables based on that message.
		It looks for 5 different message types:
		-'BAD_DATA'
		-'RC_CHANNELS_RAW'
		-'GLOBAL_POSITION_INT' (for SITL only)
		-'ATTITUDE'
		-'HEARTBEAT'
		This is non-blocking so does not gaurantee to chnage any values.
		"""
		self.msg = self.master.recv_match(blocking=False)

		if self.msg:
			#print self.msg.get_type()

			if self.msg.get_type() == "RC_CHANNELS_RAW":
				try:
					self.current_rc_channels[0] = self.msg.chan1_raw
					self.current_rc_channels[1] = self.msg.chan2_raw
					self.current_rc_channels[2] = self.msg.chan3_raw
					self.current_rc_channels[3] = self.msg.chan4_raw
					self.current_rc_channels[4] = self.msg.chan5_raw
					self.current_rc_channels[5] = self.msg.chan6_raw
				except:
					pass

				self.send_rc_overrides()

			if self.sitl == True:
				if self.msg.get_type() == "ATTITUDE":
					self.current_yaw = self.msg.yaw*180/math.pi

				if self.msg.get_type() == "GLOBAL_POSITION_INT":
					self.current_lat = self.msg.lat * 1.0e-7
					self.current_lon = self.msg.lon * 1.0e-7
					self.current_alt = self.msg.alt-self.ground_alt

	def connect_vicon(self):
		"""
		Connects to vicon. This is needed to scream vicon data.
		"""
		signal.signal(signal.SIGINT, self.signal_handler)
		self.quad = vicon.ViconPosition("pixhawk(new)@Vicon")
		self.quad.start()
		
		if self.num_vicon_objs == 2:
			self.wand = vicon.ViconPosition("Wand@Vicon")
			self.wand.start()
			while (self.get_vicon()[4] == 0 and self.get_vicon()[5] == 0 and self.get_vicon()[6] == 0) or self.get_vicon() == 0:
				pass
			print "Connected to wand location..."
		while (self.get_vicon()[1] == 0 and self.get_vicon()[2] == 0 and self.get_vicon()[3] == 0) or self.get_vicon() == 0:
                        pass
                print "Connected to pixhawk location..."

		print "Connected to vicon..."
		
	def disconnect_vicon(self):
		"""
		Properly closes vicon connection. Call this when finished using vicon.
		"""
		self.quad.stop()
		self.wand.stop()
		
	def signal_handler(self, signal, frame):
		print('Exiting controller')
		self.quad.stop()
		self.wand.stop()
		sys.exit(0)

	def connect(self):
		"""
		Connects to mavlink and vicon
		"""
		self.connect_mavlink()
		if self.sitl == False:
			self.connect_vicon()

	def close(self):
		"""
		Call at the end of all programs.
		"""
		if self.sitl == False:
			self.disconnect_vicon()

	def get_vicon(self):
		"""
		This function is currently a mess because of changing libraries for input.
		I could make the whole vicon input much cleaner but I dont have the time to
		go through and fix everything else so instead I am just fitting the new
		input to the old one.
		
		Gets vicon data in the folling format:

		if num_vicon_objs == 1:
			vicon_data()[0] = time
			vicon_data()[1] = x
			vicon_data()[2] = y
			vicon_data()[3] = z
			vicon_data()[4] = x rotation
			vicon_data()[5] = y rotation
			vicon_data()[6] = z rotation

		if num_vicon_objs == 2:
			vicon_data()[0] = time
			vicon_data()[1] = x_1
			vicon_data()[2] = y_1
			vicon_data()[3] = z_1
			vicon_data()[4] = x_2
			vicon_data()[5] = y_2
			vicon_data()[6] = z_2
			vicon_data()[7] = x_rotation_1
			vicon_data()[8] = y_rotation_1
			vicon_data()[9] = z_rotation_1 (yaw)
			vicon_data()[10] = x_rotation_2
			vicon_data()[11] = y_rotation_2
			vicon_data()[12] = z_rotation_2
		"""
		
		if self.num_vicon_objs == 1:
			lst = [0,self.quad.position[0]*1000,self.quad.position[1]*1000,self.quad.position[2]*1000,self.quad.angles[0],self.quad.angles[1],self.quad.angles[2]]
			return lst
		if self.num_vicon_objs == 2:
			lst = [0,self.quad.position[0]*1000,self.quad.position[1]*1000,self.quad.position[2]*1000,self.wand.position[0]*1000,self.wand.position[1]*1000,self.wand.position[2]*1000,self.quad.angles[0],self.quad.angles[1],self.quad.angles[2],self.wand.angles[0],self.wand.angles[1],self.wand.angles[2]]
			return lst
		return

	def set_vicon_home(self):
		"""
		Set the home coordinate for the vicon data.
		"""
		self.home_x = self.get_vicon()[1]
		self.home_y = self.get_vicon()[2]
		self.home_z = self.get_vicon()[3]

	def set_fence(min_x, max_x, min_y, max_y, min_z, max_z):
		"""
		Set the fence for the quadcopter to stay within. This currently just sets global fence variables.
		"""
		self.fence_x_min = mix_x
		self.fence_x_max = max_x
		self.fence_y_min = min_y
		self.fence_y_max = max_y
		self.fence_z_min = min_z
		self.fence_z_max = max_z

	def rc_filter(self, rc_value, rc_min, rc_max):
		"""
		Filter for the RC values to filter out RC values out of RC range. Returns filtered RC value
		"""
		if rc_value > rc_max:
			rc_value = rc_max
		if rc_value < rc_min:
			rc_value = rc_min
		return rc_value

	def send_rc_overrides(self):
		self.master.mav.rc_channels_override_send(self.master.target_system, self.master.target_component, self.current_rc_overrides[0], self.current_rc_overrides[1], self.current_rc_overrides[2], self.current_rc_overrides[3], self.current_rc_overrides[4], self.current_rc_overrides[5], 0, 0)
		#self.master.mav.file.fd.flush()

	## Set RC Channels ##
	def set_rc_roll(self, rc_value):
		rc_value = self.rc_filter(rc_value,1519-270,1519+270)
		self.current_rc_overrides[0] = rc_value

	def set_rc_pitch(self, rc_value):
		rc_value = self.rc_filter(rc_value, 1519-270,1519+270)
		self.current_rc_overrides[1] = rc_value


	def set_rc_throttle(self, rc_value):
		rc_value = self.rc_filter(rc_value, 1110, 1741)
		self.current_rc_overrides[2] =  rc_value


	def set_rc_yaw(self, rc_value):
		rc_value = self.rc_filter(rc_value, 1277, 1931)
		self.current_rc_overrides[3] = rc_value



	## Reset RC Channels ##
	def rc_roll_reset(self):
		self.current_rc_overrides[0] = 0


	def rc_pitch_reset(self):
		self.current_rc_overrides[1] = 0


	def rc_throttle_reset(self):
		self.current_rc_overrides[2] = 0


	def rc_yaw_reset(self):
		self.current_rc_overrides[3] = 0


	def rc_channel_five_reset(self):
		self.current_rc_overrides[4] = 0


	def rc_channel_six_reset(self):
		self.current_rc_overrides[5] = 0

	def rc_all_reset(self):
		self.rc_roll_reset()
		self.rc_pitch_reset()
		self.rc_throttle_reset()
		self.rc_yaw_reset()

	def rc_check_dup(self, channel, value):
		"""
		Check for duplicate RC value. This is used to check to see if the RC value is already set to the value being passed in.
		"""
		if self.v.channel_readback[channel] == value:
			return True
		return False

	def get_alt(self):
		"""
		Returns the altitude in mm in SITL
		"""
		return self.current_alt

	def get_lat(self):
		"""
		Returns the latitude of the copter in SITL
		"""
		return self.current_lat

	def get_lon(self):
		"""
		Returns the longitude of the copter in SITL
		"""
		return self.current_lon

	def get_roll(self):
		"""
		Returns the roll in radians
		"""
		return self.current_roll

	def get_yaw_radians(self):
		"""
		NOT WORKING. DO NOT USE. IMPUT HAS BEEN CHANGED.
		Returns the current yaw in radians from -pi to pi
		Works for both SITL and Vicon
		For SITL it returns that yaw givn by the copter and for the Vicon system it returns the yaw given by the Vicon
		"""
		yaw = None

		if self.sitl == True:
			yaw = self.current_yaw

		else:
			try:
				#Depending on different number of objects yaw located in different place in the vicon data stream
				if self.num_vicon_objs == 1:
					yaw = self.get_vicon()[6]*(1.0)
				if self.num_vicon_objs == 2:
					yaw = self.get_vicon()[9]*(1.0)
				self.vicon_error = False
			except:
				logging.error('Unable to get the yaw(radians) from vicon')
				yaw = None
				self.vicon_error = True
		return yaw

	def get_yaw_degrees(self):
		"""
		NOT WORKING. DO NOT USE. IMPUT HAS BEEN CHANGED.
		Returns the current yaw in degrees from 0 to 360
		Works for SITL and Vicon
		For SITL it returns that yaw givn by the copter and for the Vicon system it returns the yaw given by the Vicon
		"""
		try:
			if self.num_vicon_objs == 1:
				yaw = math.degrees((self.get_vicon()[6]*(1.0)) % ((2*math.pi)*(1.0)))*-1
			if self.num_vicon_objs == 2:
				yaw = math.degrees((self.get_vicon()[9]*(1.0)) % ((2*math.pi)*(1.0)))*-1
			self.vicon_error = False
			if yaw < 0.0:
				yaw += 360

		except:
			logging.error('Unable to get the yaw(radians) from vicon')
			yaw = None
			self.vicon_error = True

		return yaw

	def get_pitch(self):
		"""
		Returns the pitch of the copter in radians
		"""
		return self.current_pitch

	def get_position(self):
		"""
		Will return position in millimeters. (X,Y,Z)
		Use this for Vicon and SITL in the loop.
		"""
		position=[None]*3

		if self.sitl == True:
			position[0] = self.calc_sitl_distance_x()
			position[1] = self.calc_sitl_distance_y()
			position[2] = self.get_alt()

			#Assign distance with appropriate sign
			if self.get_lat() < self.home_lat:
				position[0] *= -1
			if self.get_lon() < self.home_lon:
				position[1] *= -1

		else:
			try:
				position[0] = self.get_vicon()[1]
				position[1] = self.get_vicon()[2]
				position[2] = self.get_vicon()[3]
				self.vicon_error = False
			except:
				logging.error('Unable to get position data from vicon')
				position = None
				self.vicon_error = True
		return position

	def get_distance_xy(self):
		"""
		Returns the distance traveled from home in millimeters
		"""
		if self.sitl == True:
			distance = self.calc_sitl_distance(self.home_lat, self.home_lon, selt.get_lat(), self.get_lon())
		else:
			distance = math.sqrt(self.get_position()[0]*self.getposition()[0] + self.get_position()[1]*self.get_position()[1])

		return distance

	def set_sitl_home(self):
		"""
		Sets home for SITL
		"""
		self.home_lat = self.get_lat()
		self.home_lon = self.get_lon()
		self.home_alt = self.get_alt()

	def set_home(self):
		"""
		Sets the home for he quadcopter.
		Use this for Vicon and SITL
		"""
		if self.sitl == True:
			self.set_sitl_home()
		else:
			self.set_vicon_home()

	def calc_sitl_distance(self, lat1, lon1, lat2, lon2):
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

	def calc_sitl_distance_x(self):
		"""
		Calculate x distance in SITL (lat)
		X axis is north/south (change in lat)
		"""
		return self.calc_sitl_distance(self.home_lat, self.home_lon, self.home_lat, self.get_lon())

	def calc_sitl_distance_y(self):
		"""
		Calculate y distance in SITL (lon)
		Y axis is east/west (change in lon)
		"""
		return self.calc_sitl_distance(self.home_lat, self.home_lon, self.get_lat(), self.home_lon)

