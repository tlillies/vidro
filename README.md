# vidro #

An API combing DroneAPI and Vicon streaming data to close the feedback control loop

###Resources###

* [pymavlink](https://github.com/mavlink/pymavlink)

* [Vicon Streaming](https://github.com/cfinucane/pyvicon)

* [MavProxy](https://github.com/tridge/MAVProxy)


## Contents ##

### vidro_class.py ###

This is the class for connecting to the APM and vicon. It is able to handle SITL of the APM which is useful for testing.

### position_controller.py ###

Example PID controller using rc overrides.

### sitl_.py ###

SITL implementation of vidro and override controller to close the loop.

### flight_test.py ###

Implementation of vidro and rc override controller to close the loop.

TODO
======

* fix connection for actuall hardware (Only is able to connect after connecting to GCS? only getting heartbeat before that...)
* print curses over certain intervals so loop can run full cpu
* create veiw_rc.py script
* create script for finding scripts
* tune sitl controller
* read gains from file in controller.py
* __clean!!__
