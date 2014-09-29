"""
Class for handling the PID controller
"""


from vidro_class import Vidro, ViconStreamer
import sys, math, time
import socket, struct, threading
import curses
import utm
import matplotlib.pyplot as plot

class PositionController:
	def __init__(self,vidro):
		self.vidro = vidro
		
		self.timer = time.clock()
		
		#Previous errors for calculating I and D
		self.previous_time_alt = 0
		self.I_error_alt = 0
		self.error_alt = 0

		self.previous_time_yaw = 0
		self.I_error_yaw = 0
		self.error_yaw = 0

		self.previous_time_xy = 0
		self.previous_error_pitch = 0
		self.previous_error_roll = 0
		self.D_error_roll = 0
		self.D_error_pitch = 0
		self.I_error_roll = 0
		self.I_error_pitch = 0
		self.error_pitch = 0
		self.error_roll = 0
		self.error_x = 0
		self.error_y = 0

		#Gains for PID controller
		self.alt_K_P = 0
		self.alt_K_I = 0

		self.yaw_K_P = 0
		self.yaw_K_I = 0

		self.roll_K_P = 0
		self.roll_K_I = 0
		self.roll_K_D = 0

		self.pitch_K_P = 0
		self.pitch_K_I = 0
		self.pitch_K_D = 0

	def rc_alt(self, goal_alt):
		"""
		Will send copter based off of throttle to 'goal_alt'
		"""
		#Calculate delta t and set previous time ot current time
		current_time = ((time.clock()-self.timer)*10)
		delta_t = current_time - self.previous_time_alt
		self.previous_time_alt = current_time

		#Get error
		self.error_alt = goal_alt - self.vidro.get_alt()

		#Get error I
		self.I_error_alt = self.I_error_alt + self.error_alt*delta_t

		#Print alt data
		#curses_print("Throttle RC Level: " + str(v.channel_readback['3']), 6, 1)
		#curses_print("Error: " + str(error_alt), 7, 1)
		#curses_print("Altitude:" + str(get_alt()), 8, 1)
		#curses_print("T: "+ str(int(1630+error_alt*alt_K_P+I_error_alt*alt_K_I)) + " = 1630 + " + str(error_alt*alt_K_P) + " + " + str(I_error_alt*alt_K_I), 19, 0)

		#Send RC value
		self.vidro.rc_throttle(1630 + self.error_alt*self.alt_K_P + self.I_error_alt*self.alt_K_I)

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
		self.error_yaw = goal_heading - self.vidro.get_yaw_radians()

		#Get error I
		self.I_error_yaw = self.I_error_yaw + self.error_yaw*self.delta_t

		#Print yaw data
		#curses_print("Yaw RC Level: " + str(v.channel_readback['4']), 6, 0)
		#curses_print("Error: " + str(error_yaw), 7, 0)
		#curses_print("Heading Radians: " + str(get_yaw_radians()), 8, 0)
		#curses_print("Heading Degrees: " + str(get_yaw_degrees()), 9, 0)
		#curses_print("Y: "+ str(int(1500+error_yaw*yaw_K_P+I_error_yaw*yaw_K_I)) + " = 1500 + " + str(error_yaw*yaw_K_P) + " + " + str(I_error_yaw*yaw_K_I), 20, 0)

		#Send RC value
		self.vidro.rc_yaw(1500 + self.error_yaw*self.yaw_K_P + self.I_error_yaw*self.yaw_K_I)

		return self.error_yaw


	def rc_xy(self, goal_x, goal_y):
		"""
		Sends quad copter to given x-y point
		"""
		
		#Get current heading for shifting axis
		if self.vidro.sitl == True:
			heading = self.vidro.get_yaw_degrees()
		else:
			heading = self.vidro.get_yaw_degrees()*-1

		#Calculate current position
		self.x_current = self.vidro.get_position()[0]
		self.y_current = self.vidro.get_position()[1]

		#Assign distance with appropriate sign
		if self.vidro.sitl == True:
			if self.vidro.get_lat() < home_lat:
				self.y_current *= -1
			if self.vidro.get_lon() < home_lon:
				self.x_current *= -1

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

		#Print functions for curses
		#curses_print("Pitch RC Level: " + str(v.channel_readback['2']), 11, 0)
		#curses_print("Roll RC Level: " + str(v.channel_readback['1']), 11, 1)
		#curses_print("Pitch: " + str(get_pitch()), 12, 0)
		#curses_print("Roll: " + str(get_roll()), 12, 1)
		#curses_print("X Error: " + str(round(error_x)), 15, 0)
		#curses_print("Y Error: " + str(round(error_y)), 15, 1)
		#curses_print("Roll Error: " + str(round(error_roll)), 13, 1)
		#curses_print("Pitch Error: " + str(round(error_pitch)), 13, 0)
		#curses_print("Total Error: " + str(round(total_error)), 16, 0)

		#Calculate delta-t for integration
		current_time = ((time.clock()-self.timer)*10)
		delta_t = current_time - self.previous_time_xy
		self.previous_time_xy = current_time

		#Calculate the I error for roll and pitch
		self.I_error_roll = self.I_error_roll + self.error_roll*delta_t
		self.I_error_pitch = self.I_error_pitch + self.error_pitch*delta_t

		#Calculate the D error for roll and pitch
		self.D_error_roll = (self.error_roll-self.previous_error_roll)/delta_t
		self.D_error_pitch = (self.error_pitch-self.previous_error_pitch)/delta_t

		self.previous_error_pitch = self.error_pitch
		self.previous_error_roll = self.error_roll

		#curses_print("P: " +  str(int(1540+error_pitch*pitch_K_P+I_error_pitch*pitch_K_I+D_error_pitch*pitch_K_D)) + " = 1540 + " + str(error_pitch*pitch_K_P) + " + " + str(I_error_pitch*pitch_K_I) + " + " + str(D_error_pitch*pitch_K_D), 21, 0)
		#curses_print("R: " +  str(int(1540+error_roll*roll_K_P+I_error_roll*roll_K_I+D_error_roll*roll_K_D)) + " = 1540 + " + str(error_roll*roll_K_P) + " + " + str(I_error_roll*roll_K_I) + " + " + str(D_error_roll*roll_K_D), 22, 0)

		#Send RC values
		self.vidro.rc_pitch( 1540 + (self.error_pitch*self.pitch_K_P) + (self.I_error_pitch*self.pitch_K_I) + (self.D_error_pitch*self.pitch_K_D) )
		self.vidro.rc_roll(  1540 + (self.error_roll*self.roll_K_P) + (self.I_error_roll*self.roll_K_I) + (self.D_error_roll*self.roll_K_D) )
		

vidro = Vidro(True, 115200,"127.0.0.1:14551")
vidro.connect()
controller = PositionController(vidro)
while vidro.current_rc_channels[4] < 1600:
	controller.rc_alt()
	controller.rc_yaw()
	controller.rc_xy()
	vidro.get_mavlink()
	time.sleep(.01)
vidro.close()
