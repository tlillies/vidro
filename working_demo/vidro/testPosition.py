import vicon
import signal
import sys
import time

def signal_handler(signal, frame):
    global quad
    global wand
    print('Exiting controller')
    quad.stop()
    wand.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
quad = vicon.ViconPosition("pixhawk(new)@Vicon")
quad.start()

wand = vicon.ViconPosition("Wand@Vicon")
wand.start()

while (1):
    print "QUAD" + str(quad.position)
    print "WAND" + str(wand.position)




