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

### sitl_controller_test.py ###

SITL implementation of vidro and override controller to close the loop.

### flight_test.py ###

Implementation of vidro and rc override controller to close the loop.

TODO
======

* <del>fix connection for actuall hardware (Only is able to connect after connecting to GCS? only getting heartbeat before that...)</del>
* <del>print curses over certain intervals so loop can run full cpu</del> NEED TO TEST
* <del>create veiw_rc.py script</del> NEED TO TEST
* <del>create script for finding rc limits</del> NEED TO TEST
* tune sitl controller
* <del>read gains from file in controller.py</del> NEED TO TEST
* __clean!!__
