import transformations
import vrpn
import math
import time

def callback_quad(userdata, packet):
	#print packet
    quat = packet['quaternion']
    fixed_axes = (quat[3], quat[0], quat[1], quat[2])
        
    #print fixed_axes
    angles = transformations.euler_from_quaternion(fixed_axes, 'rxyz')
    #print angles
    rad2deg = 180 / math.pi
        
    position = packet['position']
    pos_angles = (angles[1]*rad2deg, angles[0]*rad2deg, angles[2]*rad2deg)
    print "QUAD: X: %f \tY: %f \tZ: %f \tRoll: %f \tPitch: %f \tYaw: %f" % \
         (position[0], position[1], position[2], pos_angles[0], pos_angles[1], pos_angles[2])
      
def callback_wand(userdata, packet):
	#print packet
    quat = packet['quaternion']
    fixed_axes = (quat[3], quat[0], quat[1], quat[2])
        
    #print fixed_axes
    angles = transformations.euler_from_quaternion(fixed_axes, 'rxyz')
    #print angles
    rad2deg = 180 / math.pi
        
    position = packet['position']
    pos_angles = (angles[1]*rad2deg, angles[0]*rad2deg, angles[2]*rad2deg)
    print "WAND: X: %f \tY: %f \tZ: %f \tRoll: %f \tPitch: %f \tYaw: %f" % \
         (position[0], position[1], position[2], pos_angles[0], pos_angles[1], pos_angles[2])
	
quad=vrpn.receiver.Tracker("pixhawk(new)@vicon")
quad.register_change_handler("position", callback_quad, "position")

wand=vrpn.receiver.Tracker("Wand@vicon")
wand.register_change_handler("position", callback_wand, "position")

while 1:
    quad.mainloop()
    wand.mainloop()
