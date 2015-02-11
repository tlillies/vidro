import sys, math, time
from droneapi.lib import VehicleMode
from pymavlink import mavutil

global api
api = local_connect()
global v
v = api.get_vehicles()[0]
print "API Connected..."

v.flush()

"Waiting for flight data to stream...."

time.sleep(20)

print "Done waiting...."
