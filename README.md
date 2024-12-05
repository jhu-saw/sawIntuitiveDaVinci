# sawIntuitiveDaVinci

This SAW component contains code for interfacing with the da Vinci medical robot using the research API (Intuitive Surgical Inc, https://www.intuitivesurgical.com/).  It compiles on Windows and Linux.  It requires the binaries for the ISI Research API as well as a da Vinci robot with the research API activated.  You must have a research agreement in place with ISI for these requirements.

The `ros` folder contains code for a ROS node that interfaces with the sawIntuitiveDaVinci component and publishes the 3D transformations of each arm and setup joints as well as the joint states (position, velocity and effort) and some console events (foot pedals).  It also broadcasts transformations for `tf2`.  To build the ROS node, make sure you use `catkin build`.

If needed, one can also add OpenIGTLink support using sawOpenIGTLink with a few simple configuration files.

# Links
 * License: http://github.com/jhu-cisst/cisst/blob/master/license.txt
 * JHU-LCSR software: http://jhu-lcsr.github.io/software/

# Dependencies
 * cisst libraries: https://github.com/jhu-cisst/cisst
 * Qt for user interface
 * ROS with cisst-ros (optional): https://github.com/jhu-cisst/cisst-ros
 * OpenIGTLink with sawOpenIGTLink (optional): https://github.com/jhu-saw/sawOpenIGTLink

# Compilation

If you are using this package on Linux with ROS, please download the code using the `.rosinstall` file in this repository and then compile using `catkin build` (**NOT** `catkin_make`).  The process is very similar to the compilation for the dVRK code: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild#catkin-build-and-rosinstall but for `wstool`, use:
```sh
wstool merge https://raw.githubusercontent.com/jhu-saw/sawIntuitiveDaVinci/master/ros/isi.rosinstall
```

# Running the examples

## Main example

The main example provided is `sawIntuitiveDaVinciConsoleQt`.  You need to make sure your computer is on the same subnet as the da Vinci, i.e. you should be able to ping 10.0.0.5.  This example can be compiled on Linux or Windows.

## ROS

Please read the section above to make sure you can communicate with the da Vinci.  The ROS node is `intuitive_da_vinci` and can be found in the package `intuitive_da_vinci`:
```sh
rosrun intuitive_da_vinci intuitive_da_vinci
```

The topic names are based on the arm names:
```sh
rostopic list
/ECM1/body/measured_cv
/ECM1/follow_mode
/ECM1/measured_cp
/ECM1/measured_js
/MTML1/body/measured_cv
/MTML1/clutch
/MTML1/follow_mode
/MTML1/gripper/measured_js
/MTML1/measured_cp
/MTML1/measured_js
/MTML1/select
/MTMR1/body/measured_cv
/MTMR1/clutch
/MTMR1/follow_mode
/MTMR1/gripper/measured_js
/MTMR1/measured_cp
/MTMR1/measured_js
/MTMR1/select
/PSM1/body/measured_cv
/PSM1/follow_mode
/PSM1/jaw/measured_js
/PSM1/measured_cp
/PSM1/measured_js
/PSM2/body/measured_cv
/PSM2/follow_mode
/PSM2/jaw/measured_js
/PSM2/measured_cp
/PSM2/measured_js
/PSM3/body/measured_cv
/PSM3/follow_mode
/PSM3/jaw/measured_js
/PSM3/measured_cp
/PSM3/measured_js
/SUJ/ECM1/measured_cp
/SUJ/ECM1/measured_js
/SUJ/PSM1/measured_cp
/SUJ/PSM1/measured_js
/SUJ/PSM2/measured_cp
/SUJ/PSM2/measured_js
/SUJ/PSM3/measured_cp
/SUJ/PSM3/measured_js
/console/camera
/console/camera_quick_tap
/console/clutch
/console/clutch_quick_tap
/console/follow_mode
/console/head_in
/console/head_out
/console/masters_as_mice
/console/operator_present
```

You can also visualize the tf2 output using:
```sh
rosrun tf2_tools view_frames.py
evince frames.pdf
```

## OpenIGTLink

You will need to compile sawOpenIGTLink first: https://github.com/jhu-saw/sawOpenIGTLink.  This should work on both Linux and Windows.

Then you can start streaming data from the da Vinci using IGTL using a
couple of configuration files, one to load the sawOpenIGTLink
component (`manager-igtl.json`) and one to configure the IGTL bridge
(`igtl-isi.json`).  Both files can be found in the `share` folder.

To start any of the examples with the IGTL bridge, use the `-m` command line option (this option works with both example programs, `sawIntuitiveDaVinciConsoleQt` and `intuitive_da_vinci/intuitive_da_vinci`:
```sh
sawIntuitiveDaVinciConsoleQt -m manager-igtl.json
```

To display all the data being streamed, you can use the following application (provided along sawOpenIGTLink):
```sh
igtl_receive localhost 18944
```

If you start the same application with a dummy device name, it will listen for a bit and list the devices it received data for.  For example:
```sh
igtl_receive localhost 18944 doesnotexist
```
will report:
```
ECM1/measured_cp
ECM1/measured_cv
ECM1/measured_js
MTML1/measured_cp
MTML1/measured_cv
MTML1/measured_js
MTMR1/measured_cp
...
```

Then you can select a specific device using:
```sh
igtl_receive localhost 18944 PSM1/measured_cp
```
will report something like:
```
Device name: PSM1/measured_cp
Time stamp: 26.185951038
Receiving TRANSFORM data type.
=============
-0.76992, -0.337651, 0.541492, -0.0135498
0.401987, -0.915644, 0.000608893, 0.00900606
0.495609, 0.218142, 0.840705, 0.058111
0, 0, 0, 1
```
