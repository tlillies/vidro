import math

heading = 123

goal_x = 10
goal_y = 10

x_current = 0
y_current = 0

x_error = goal_x - x_current
y_error = goal_y - y_current

#Distance from current point to goal point
total_error = math.sqrt(x_error*x_error+y_error*y_error)

print "total_error: " + str(total_error)
	
#Angle on x-y axis
waypoint_angle = math.degrees(math.atan(x_error/y_error))

print "waypoint_angle: " + str(waypoint_angle)
	
#Distance between heading and angle to waypoint
angle_difference = math.fabs(waypoint_angle-heading)

print "angle_difference: " + str(angle_difference)
	
#Make angle between 0-90
angle_difference = angle_difference % 90.0

print "angle_difference: " + str(angle_difference)
	
#Find percentage of roll
percent_roll = (100.0/90.0) * angle_difference
	
print 100.0/90.0
#Find percentage of pitch
percent_pitch = 100.0 - percent_roll

print "Percent Pitch: " + str(percent_pitch)
print "Percent Roll: " + str(percent_roll)
