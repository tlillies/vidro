#--HELPFUL STUFF--

#THIS IS NOT CODE THAT IS MADE TO BE RUN. ONLY HERE AS A RESOURCE

#DRONE API DOCUMENTATION:      http://diydrones.github.io/droneapi-python/
#DRONE API SOURCE:             https://github.com/diydrones/droneapi-python
#MAVLINK COMMON MESSAGES:      https://pixhawk.ethz.ch/mavlink/

fgfs --aircraft=Generic --generic=socket,out,50,127.0.0.1,49005,udp,ardupilot --generic=socket,in,50,127.0.0.1,49000,tcp,ardupilot --in-air --altitude=10 --vc=90 --heading=300 --timeofday=noon
fgfs --aircraft=arducopter --native-fdm=socket,out,50,127.0.0.1,49005,udp --generic=socket,in,50,127.0.0.1,49000,udp,quadhil --vc=90 --heading=300 --timeofday=noon --altitude=0

mavproxy.py =/dev/ttyUSB0 --out 127.0.0.1:49000

xdg-open .

#Start MavLink: mavproxy.py --master=/dev/ttyUSB0 --baudrate=5760

#g++ -I /home/tom/RECUV/BOOST/boost_1_55_0/ tom_test.cpp -o tom_test -L~/RECUV/BOOST/boost_1_55_0/stage/lib/ -llibboost_thread-mt.so.1.53.0 -llibboost_system-mt.so.1.53.0 -L. -lViconDataStreamSDK_CPP -lDebugServices

#g++ -I /home/tom/RECUV/BOOST/boost_1_55_0/ tom_test.cpp -o tom_test ~/RECUV/BOOST/boost_1_55_0/stage/lib/libboost_thread.a ~/RECUV/BOOST/boost_1_55_0/stage/lib/libboost_system.a -L. -lViconDataStreamSDK_CPP -lDebugServices

#Mode: 65536 ????

#Message Factory Documentation
def message_factory(self):
        """
        Returns an object that can be used to create 'raw' mavlink messages that are appropriate for this vehicle.
        These message types are defined in the central Mavlink github repository.  For example, a Pixhawk understands
        the following messages: (from https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/pixhawk.xml).

          <message id="153" name="IMAGE_TRIGGER_CONTROL">
               <field type="uint8_t" name="enable">0 to disable, 1 to enable</field>
          </message>

        The name of the factory method will always be the lower case version of the message name with _encode appended.
        Each field in the xml message definition must be listed as arguments to this factory method.  So for this example
        message, the call would be:

        msg = vehicle.message_factory.image_trigger_control_encode(True)
        vehicle.send_mavlink(msg)
        """
        return self.__module.master.mav
        

#EXAMPLE OF MESSAGE FACTORY FROM "small_demo.py"
msg = v.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, 0, 0, 0, 0, 1, 0, 0)
print "Created msg: %s" % msg
v.send_mavlink(msg)

#MAV_CMD_CONDITION_YAW	Reach a certain target angle.

#Mission Param 1		target angle: [0-360], 0 is north
#Mission Param 2		speed during yaw change:[deg per second]
#Mission Param 3		direction: negative: counter clockwise, positive: clockwise [-1,1]
#Mission Param 4		relative offset or absolute angle: [ 1,0]
#Mission Param 5		Empty
#Mission Param 6		Empty
#Mission Param 7		Empty


#MAV_CMD_DO_SET_MODE	Set system mode.

#Mission Param 1		Mode, as defined by ENUM MAV_MODE
#Mission Param 2		Custom mode - this is system specific, please refer to the individual autopilot specifications for details.
#Mission Param 3		Empty
#Mission Param 4		Empty
#Mission Param 5		Empty
#Mission Param 6		Empty
#Mission Param 7		Empty


