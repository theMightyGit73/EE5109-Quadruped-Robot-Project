# EE5109-Quadruped-Robot-Project

## Overview
This project implements a quadruped robot simulation using ROS (Robot Operating System) and Gazebo. The robot model is based on the NotSpot design, which is inspired by the Boston Dynamics Spot robot.

## Features
- Full 3D simulation in Gazebo
- Various locomotion gaits including trot and crawl
- ROS control integration for joint control
- Navigation stack integration with SLAM capability
- Inverse kinematics for leg movement
- Different controllers (PID, LQR)

## Structure
- **notspot**: Main package that integrates all components
- **notspot_controller**: Contains the robot controller and locomotion algorithms
- **notspot_description**: URDF model and visual meshes for the robot
- **notspot_gazebo**: Gazebo simulation configuration
- **notspot_joystick**: Teleoperation interfaces
- **notspot_navigation**: Navigation and mapping capabilities

## Getting Started
1. Clone this repository into your ROS workspace's src directory
2. Install dependencies: `rosdep install --from-paths src --ignore-src -r -y`
3. Build the workspace: `catkin_make`
4. Source your workspace: `source devel/setup.bash`
5. Launch the simulation: `roslaunch notspot simulation.launch`

## Controls
- Use the keyboard teleop node for basic movement
- For advanced control, refer to the controller documentation

## Contributors
- theMightyGit73

## License
This project is part of the EE5109 course work.
