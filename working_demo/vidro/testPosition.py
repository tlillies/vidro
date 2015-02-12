import vicon
import signal
import sys
import time

v = 0
def signal_handler(signal, frame):
    global v
    print('Exiting controller')
    v.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
v = Position.ViconPosition("test1@Vicon")
v.start()
while (1):
    print 'Z: %1.4f' % v.position[2]
    #time.sleep(1)




