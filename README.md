# reachy_ros_animator
A ROS node to load and play animations to animate CoLab Reachy's arms.

In this repository, you fill find:
* A ROS package, reachy_ros_animator, containing a ROS node, sequencer.py
* A script, animate, to publish a sequence_animation topic that will induce the ROS node to animate Reachy's arms
* Sample animation files

##Documentation

###Installation

Change directory to your local catkin_ws/src directory.  
Clone the reachy_ros_animator repo.

> $ git clone https://github.com/CircuitLaunch/reachy_ros_animator

Change directory to catkin_ws and make.

> $ catkin_make  
> $ source devel/setup.bash

### Running the node

Assuming that roscore has already been launched, run the node.

> $ rosrun reachy_ros_animator sequencer.py

### Playing animations

Change directory to the scripts folder within the local reachy_ros_animator repo, and execute animate passing one of the animation files.

> $ cd src/reachy_ros_animator/scripts  
> $ ./animate right_wave_animation