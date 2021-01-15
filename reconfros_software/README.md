# ReconfROS Software

The folder can be considered as ROS package, which contains the software reference implementation of the image processing pipeline and tools for the simulation environment and robot control.

## Package structure

This package provides two main nodes for the trail follower:

1. **final_pipeline:** Implementation of the computer vision pipeline which provides a target angle for following the trail 
2. **navigation:** Provides commands for the motor driver based on the target angle

The package provides two simulations where the complete trail follower setup can be tested.
It is recommended that you use the **small simulation** unless you have a very powerful computer.

The node `cv_node` can be used to compare different approaches for the trail extraction.

## Prerequisites

- ROS installation on your system (http://wiki.ros.org/melodic/Installation/Ubuntu)
- A catkin workspace (http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
- ROS packages cv_bridge, image_transport, uos_tools, ceres_robot, sick_tim (ros-melodic-cv-bridge, ros-melodic-image-transport, https://github.com/uos/uos_tools, https://github.com/uos/ceres_robot, https://github.com/uos/sick_tim)

## Installation

1. Copy this folder in the `src` directory of your catkin workspace
2. Build your workspace (with `catkin_make`)

## Running

### Without simulation (For outdoor testing)

`roslaunch software_ms1 trail_follower.launch`

This command launches the complete trail follower setup including `rqt_reconfigure`.

### With simulation

`roslaunch software_ms1 demo.launch`

This command launches the small simulation and the complete trail follower setup including `rqt_reconfigure`.
You can also start the small simulation only with:

`roslaunch software_ms1 small_simulation.launch`

Or the big simulation with:

`roslaunch software_ms1 simulation.launch`

### CV Prototype

`rosrun software_ms1 cv_node`