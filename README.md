# sawIntuitiveDaVinci

This SAW component contains code for interfacing with the da Vinci medical robot using the research API (Intuitive Surgical Inc, https://www.intuitivesurgical.com/).  It compiles on Windows and Linux.  It requires the binaries for the ISI Research API as well as a da Vinci robot with the research API activated.  You must have a research agreement in place with ISI for these requirements.

The `ros` folder contains code for a ROS node that interfaces with the sawIntuitiveDaVinci component and publishes the 3D transformations of each arm and setup joints as well as the joint states (position, velocity and effort) and some console events (foot pedals).  It also broadcasts transformations for `tf2`.  To build the ROS node, make sure you use `catkin build`.

If needed, one can also add OpenIGTLink support using sawOpenIGTLink (contact the sawIntuitiveDaVinci developers if you need help with this).

# Links
 * License: http://github.com/jhu-cisst/cisst/blob/master/license.txt
 * JHU-LCSR software: http://jhu-lcsr.github.io/software/

# Dependencies
 * cisst libraries: https://github.com/jhu-cisst/cisst
 * Qt for user interface
 * ROS (optional)

# Running the examples

## Main example

The main example provided is `sawNDIIntuitiveDaVinciQtExample`.  You need to make sure your computer is on the same subnet as the da Vinci, i.e. you should be able to ping 10.0.0.5

## ROS

Please read the section above to make sure you can communicate with the da Vinci.  The ROS node is `isi_console` and can be found in the package `isi_ros`:
```sh
rosrun isi_ros isi_console
```

The topic names are based on the arm names:
```sh
/isi/ECM1/position_cartesian_current
/isi/ECM1/state_joint_current
/isi/ECM1/twist_body_current
/isi/MTML1/position_cartesian_current
/isi/MTML1/state_joint_current
/isi/MTML1/twist_body_current
/isi/MTMR1/position_cartesian_current
/isi/MTMR1/state_joint_current
/isi/MTMR1/twist_body_current
/isi/PSM1/position_cartesian_current
/isi/PSM1/state_joint_current
/isi/PSM1/twist_body_current
/isi/PSM2/position_cartesian_current
/isi/PSM2/state_joint_current
/isi/PSM2/twist_body_current
/isi/PSM3/position_cartesian_current
/isi/PSM3/state_joint_current
/isi/PSM3/twist_body_current
/isi/SUJ/ECM1/position_cartesian_current
/isi/SUJ/ECM1/state_joint_current
/isi/SUJ/PSM1/position_cartesian_current
/isi/SUJ/PSM1/state_joint_current
/isi/SUJ/PSM2/position_cartesian_current
/isi/SUJ/PSM2/state_joint_current
/isi/SUJ/PSM3/position_cartesian_current
/isi/SUJ/PSM3/state_joint_current
```

You can also visualize the tf2 output using:
```sh
rosrun tf2_tools view_frames.py
evince frames.pdf
```
