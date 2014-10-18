"""
Class for handling the PID controller
"""
from vidro_class import Vidro, ViconStreamer
import sys, math, time
import socket, struct, threading
import matplotlib.pyplot as plot

class PositionController:
	def __init__(self,vidro):
		self.vidro = vidro

		self.timer = time.clock()

		#Previous errors for calculating I and D
		self.previous_time_alt = (time.clock()-self.timer)*10
		self.I_error_alt = 0
		self.D_error_alt = 0
		self.previous_error_alt = None
		self.error_alt = 0


		self.previous_time_yaw = (time.clock()-self.timer)*10
		self.I_error_yaw = 0
		self.D_error_yaw = 0
		self.error_yaw = 0
		self.previous_error_yaw = None

		self.previous_time_xy = (time.clock()-self.timer)*10
		self.previous_error_pitch = None
		self.previous_error_roll = None
		self.D_error_roll = 0
		self.D_error_pitch = 0
		self.I_error_roll = 0
		self.I_error_pitch = 0
		self.error_pitch = 0
		self.error_roll = 0
		self.error_x = 0
		self.error_y = 0

		#Gains for PID controller
		self.alt_K_P = .005
		self.alt_K_I = .00003
		self.alt_K_D = 0

		self.yaw_K_P = 10
		self.yaw_K_I = 0
		self.yaw_K_D = 0

		self.roll_K_P = .01
		self.roll_K_I = .0006
		self.roll_K_D = .05

		self.pitch_K_P = .01
		self.pitch_K_I = .0006
		self.pitch_K_D = .05

		#Base RC values
		if self.vidro.sitl == True:
			self.base_rc_roll = 1535
			self.base_rc_pitch = 1535
			self.base_rc_throttle = 1370
			self.base_rc_yaw = 1470

			#self.gains_file_path = '/home/tom/RECUV/vidro/vidro/gains_sitl.txt'
			self.gains_file_path = '/home/recuv/sources/vidro/gains_sitl.txt'
		else:
			self.base_rc_roll = 1620
			self.base_rc_pitch = 1620
			self.base_rc_throttle = 1430
			self.base_rc_yaw = 1520

			self.gains_file_path = '/home/tom/RECUV/vidro/vidro/gains.txt'
			#self.gains_file_path = '/home/recuv/sources/vidro/gains.txt'


	def update_gains(self):
		"""
		Update gains from the gains file
		"""
		#Get fill lines with the lines from the gain file
		gainfile = open(self.gains_file_path, "r")
		lines = gainfile.readlines()
		gainfile.close()

		#Set the gain files from the values from the file
		self.alt_K_P = float(lines[1])
		self.alt_K_I = float(lines[3])
		self.alt_K_D = float(lines[5])
		self.yaw_K_P = float(lines[7])
		self.yaw_K_I = float(lines[9])
		self.yaw_K_D = float(lines[11])
		self.roll_K_P = float(lines[13])
		self.roll_K_I = float(lines[15])
		self.roll_K_D = float(lines[17])
		self.pitch_K_P = float(lines[19])
		self.pitch_K_I = float(lines[21])
		self.pitch_K_D = float(lines[23])

	def rc_alt(self, goal_alt):
		"""
		Will send copter based off of throttle to 'goal_alt'
		"""

		#Calculate delta t and set previous time ot current time
		current_time = ((time.clock()-self.timer)*10)
		delta_t = current_time - self.previous_time_alt
		self.previous_time_alt = current_time

		#Get error
		try:
			self.error_alt = goal_alt - self.vidro.get_position()[2]
		except:
			self.vidro.set_rc_throttle(self.base_rc_throttle)
			return

		#Get error I
		if abs(self.error_alt) < 1000:
			self.I_error_alt = self.I_error_alt + self.error_alt*delta_t
		else:
			pass

		#Get error D
		if self.previous_error_alt == None:
			self.previous_error_alt = self.error_alt

		if self.error_alt != self.previous_error_alt:
			self.D_error_alt = (self.error_alt-self.previous_error_alt)/delta_t
			self.previous_error_alt = self.error_alt

		#Send RC value
		self.vidro.set_rc_throttle(round(self.base_rc_throttle + self.error_alt*self.alt_K_P + self.I_error_alt*self.alt_K_I + self.D_error_alt*self.alt_K_D))

		return self.error_alt

	def rc_yaw(self, goal_heading):
		"""
		Sends quad to given yaw
		Imput is in radians from -pi to pi
		"""

		#Get rid of bad inputs
		if goal_heading > math.pi or goal_heading < math.pi*-1:
			return 0

		#Calculate delta t and set previous time ot current time
		current_time = ((time.clock()-self.timer)*10)
		delta_t = current_time - self.previous_time_yaw
		self.previous_time_yaw = current_time

		#Get error
		try:
			yaw = self.vidro.get_yaw_radians()
		except:
			self.vidro.set_rc_yaw(self.base_rc_yaw)
			return

		self.error_yaw = goal_heading - yaw

		if abs(goal_heading - (yaw+2*math.pi)) < abs(self.error_yaw):
			self.error_yaw = goal_heading - (yaw+2*math.pi)
		if abs(goal_heading - (yaw-2*math.pi)) < abs(self.error_yaw):
			self.error_yaw = goal_heading - (yaw-2*math.pi)

		self.error_yaw = self.error_yaw * -1

		#Get error I
		self.I_error_yaw = self.I_error_yaw + self.error_yaw*delta_t

		#Get error D
		if self.previous_error_yaw == None:
			self.previous_error_yaw = self.error_yaw

		if self.previous_error_yaw != self.error_yaw:
			self.D_error_yaw = (self.error_yaw-self.previous_error_yaw)/delta_t
			self.previous_error_yaw = self.error_yaw

		#Send RC value
		self.vidro.set_rc_yaw(self.base_rc_yaw + self.error_yaw*self.yaw_K_P + self.I_error_yaw*self.yaw_K_I + self.D_error_yaw*self.yaw_K_D)

		return self.error_yaw


	def rc_xy(self, goal_x, goal_y):
		"""
		Sends quad copter to given x-y point
		"""

		#Get current heading for shifting axis
		if self.vidro.sitl == True:
			heading = self.vidro.get_yaw_degrees()
		else:
			try:
				heading = self.vidro.get_yaw_degrees()
			except:
				self.vidro.set_rc_pitch(self.base_rc_pitch)
				self.vidro.set_rc_roll(self.base_rc_roll)
				return

		#Calculate current position
		try:
			self.x_current = self.vidro.get_position()[0]
			self.y_current = self.vidro.get_position()[1]
		except:
			self.vidro.set_rc_pitch(self.base_rc_pitch)
			self.vidro.set_rc_roll(self.base_rc_roll)
			return

		#Calculate the error in the x-y(lat/lon) axis
		self.error_x = goal_x - self.x_current * 1.0
		self.error_y = goal_y - self.y_current * 1.0

		#Make error not zero so you don't get a divid by zero error (Need to tes to see if needed)
		if self.error_x == 0:
			self.error_x += .000000000001

		if self.error_y == 0:
			self.error_y += .000000000001

		#Total error from current point to goal point
		total_error = math.sqrt(self.error_x*self.error_x+self.error_y*self.error_y)

		#Angle on x-y(lat/lon) axis to point
		waypoint_angle = math.degrees(math.atan2(self.error_y,self.error_x))

		#Calculate the offset of the vehicle from the x-y (lat-lon) axis
		vehicle_angle = 90 - (waypoint_angle + heading)

		#Calculate the error for the roll and pitch
		self.error_roll = total_error * math.sin(math.radians(vehicle_angle))
		self.error_pitch = total_error * math.cos(math.radians(vehicle_angle))*-1

		#Calculate delta-t for integration
		current_time = ((time.clock()-self.timer)*10)
		delta_t = current_time - self.previous_time_xy
		self.previous_time_xy = current_time

		#Calculate the I error for roll and pitch
		if abs(self.error_roll) < 300:
			self.I_error_roll = self.I_error_roll + self.error_roll*delta_t
		else:
			self.I_eror_roll = 0

		if abs(self.error_pitch) < 300:
			self.I_error_pitch = self.I_error_pitch + self.error_pitch*delta_t
		else:
			self.I_error_pitch = 0

		#Calculate the D error for roll and pitch
		if self.previous_error_roll == None:
			self.previous_error_roll = self.error_roll
		if self.previous_error_pitch == None:
			self.previous_error_pitch = self.error_pitch

		if self.previous_error_roll != self.error_roll:
			self.D_error_roll = (self.error_roll-self.previous_error_roll)/delta_t
			self.previous_error_roll = self.error_roll

		if self.previous_error_pitch != self.error_pitch:
			self.D_error_pitch = (self.error_pitch-self.previous_error_pitch)/delta_t
			self.previous_error_pitch = self.error_pitch

		#Send RC values
		self.vidro.set_rc_pitch( self.base_rc_pitch + (self.error_pitch*self.pitch_K_P) + (self.I_error_pitch*self.pitch_K_I) + (self.D_error_pitch*self.pitch_K_D) )
		self.vidro.set_rc_roll(  self.base_rc_roll + (self.error_roll*self.roll_K_P) + (self.I_error_roll*self.roll_K_I) + (self.D_error_roll*self.roll_K_D) )
