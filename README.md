# reachy_ros_animator
A ROS node to load and play animations to animate CoLab Reachy's arms.

In this repository, you fill find:
* A ROS package, reachy_ros_animator, containing a ROS node, sequencer.py
* A script, animate, to publish a sequence_animation topic that will induce the ROS node to animate Reachy's arms
* Sample animation files


## Documentation

### Installation

Change directory to your local catkin_ws/src directory.  
Clone the reachy_ros_animator repo.

    $ git clone https://github.com/CircuitLaunch/reachy_ros_animator

Change directory to catkin_ws and make.

    $ catkin_make  
    $ source devel/setup.bash

### Running the node

Assuming that roscore has already been launched, run the node.

    $ rosrun reachy_ros_animator sequencer.py

### Playing animations

Change directory to the scripts folder within the local reachy_ros_animator repo, and execute animate passing one of the animation files.

    $ cd src/reachy_ros_animator/scripts  
    $ ./animate right_wave_animation

## How it works

### tl;dr;

The reachy_ros_animator/sequencer.py node is a python script containing two classes. The Sequencer class is responsible for establishing and maintaining a connection to Reachy, subscribing to the ROS 'sequence_animation' topic, loading and parsing animation files, and instantiating the second class, Player, to handle each animation sequence.

The Player class is a derivative of the python Thread class, and loops through the animation sequence, submitting goal_position messages to Reachy via reachy.goto().

### Format of an animation sequence

Animation keys are listed one per line and may be any of the following. Each line is a series of whitespace separated tokens of the following format, or a blank line.

    <key_frame_duration> <cmd> <parameter*>  

      <key_frame_duration> -- floating point time in seconds before  
                              the next key is executed  
      <cmd>                -- one of dxl, load, or #  
      <parameter*>         -- variable space separated list of  
                             parameters

#### Command reference

dxl -- set actuator goal_position  

    <key_frame_duration> dxl <reachy_part_name> <goal_position> <slerp_time>  
      <reachy_part_name> -- string name of the reachy part  
      <goal_position>    -- floating point goal angle in degrees  
      <slerp_time>       -- the period over which to actuate

load -- load an animation from a file

    <key_frame_duration> load <full_filepatht>  
      <full_filepath>    -- the full path name of the animation  
                           file to load (whitespace is not  
                           currently permitted)

&#35; -- comment (can serve to introduce pauses in the animation)

    <key_frame_durationt> # <comment>

#### Example animation file

    0 dxl left_arm.shoulder_pitch -90 2  
    0 dxl left_arm.shoulder_roll 0 1  
    0 dxl left_arm.hand.wrist_pitch 45 1  
    0 dxl left_arm.arm_yaw 0 1  
    0 dxl left_arm.hand.wrist_roll 0 1  
    1 dxl left_arm.hand.gripper 0 1  
    1 dxl left_arm.elbow_pitch -90 1.5  
    0 dxl left_arm.hand.forearm_yaw 45 1.75  
    1 dxl left_arm.hand.wrist_pitch -45 1  
    0 dxl left_arm.hand.wrist_pitch -30 0.5  
    1 dxl left_arm.arm_yaw -30 1.75  
    1 dxl left_arm.hand.forearm_yaw -45 1.75  
    1 dxl left_arm.arm_yaw 30 1.75  
    1 dxl left_arm.hand.forearm_yaw 45 1.75  
    1 dxl left_arm.arm_yaw -30 1.75  
    1 dxl left_arm.hand.forearm_yaw -45 1.75  
    1 dxl left_arm.arm_yaw 30 1.75  
    1 dxl left_arm.hand.forearm_yaw 45 1.75  
    1 dxl left_arm.arm_yaw -30 1.75  
    1 dxl left_arm.hand.forearm_yaw -45 1.75  
    0 dxl left_arm.shoulder_pitch 0 2  
    0.5 dxl left_arm.arm_yaw 0 1.5  
    0.5 dxl left_arm.elbow_pitch 0 1.5  
    0 dxl left_arm.hand.forearm_yaw 0 2  
    1 dxl left_arm.hand.wrist_pitch 0 2  
    0 dxl left_arm.hand.wrist_roll 0 1  
    0 load /home/ununtu/catkin_ws/src/reachy_node_animator/scripts/left_wave_animation

### Triggering an animation

An animation is triggered by publishing a sequence_animation topic, passing the sequence as a string.

The scripts/animate script contains an example of how to publish such a message using the command line.

    rostopic pub -1 sequence_animation std_msgs/String "0 load $PWD/$1"