Change log
==========

2.0.0 (2021-04-13)
==================

* API changes:
  * Renamed commands ROS topics for CRTK
  * Some of *cisstMultiTask* symbols renamed can be found in `crtk-port/members-saw-intuitive-da-vinci.dict` and https://github.com/jhu-cisst/cisst/blob/master/utils/crtk-port/crtk-commands.dict
  * ROS new names can be found in https://github.com/jhu-cisst/cisst/blob/master/utils/crtk-port/crtk-ros-commands.dict
* Deprecated features:
  * See API changes above
* New features:
  * Added example for sawOpenIGTLink
  * More events supported: follow mode, operator present, MTM arm clutch
* Bug fixes:
  * Better support for ROS node, including `__ns:=` to set namespace
  * Added `-no-pie` for executable linker to support isi-api compiled without `-fPIC`

1.2.0 (2019-04-10)
==================

* API changes:
  * Joint state for jaw/gripper is now separated from joints used in kinematic chain
  * Requires isi-api 1.0.6!
* Deprecated features:
  * None
* New features:
  * Added support for sawSocketStreamer
  * New ROS topics for jaw/gripper state
  * ROS publishes console events
* Bug fixes:
  * None

1.1.0 (2018-05-18)
==================

* API changes:
  * Requires isi-api 1.0.6!
* Deprecated features:
  * None
* New features:
  * Added system messages and interval statistics
  * ROS/catkin build
  * ROS topics and tf2
  * New Qt widgets with moving/reference frame
  * Get joint efforts from isi-api
* Bug fixes:
  * None
