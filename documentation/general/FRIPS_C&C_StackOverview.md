# FRIPS C&C Stack Overview

The main tasks of Rover Mega Controller software (Rover Controller software running on Arduino Mega) are:

1. Listen for commands (RPM/RPS setpoints for wheels) from FRIPS C&C Stack.
2. Control rotational velocities of all wheels (RPM/RPS) using PI(D) controllers.
3. Provide feedback on instantaneous wheel angular velocities (RPM/RPS) to FRIPS C&C Stack.

The FRIPS C&C (Command & Control) Stack is a software, consisting of a group of ROS2 nodes, whose purpose is to:

1. Transform user input (Location) into rover commands for Rover Mega Controller
2. Control, Manage and Supervise all processes require to perform tasks laid out by the point above.

The figure below illustrates all nodes in FRIPS C&C Stack.

![Illustration of all nodes in FRIPS C&C Stack](/documentation/assets/ROS_Infrastructure.png)

The ROS2 nodes are a for of container that run a piece of software and abstracts certain aspects of the system (from a point of view of the software it runs) such as: communication, timers, lifecycle, loggings, etc.

The nodes in FRIPS C&C Stack were designed with simplicity and robustness in mind; each node performs only one task (or relatively few simple tasks) and performs it well.

Following is a short overview of nodes in FRIPS C&C Stack and the tasks they perform:

`Rover Interface Node`:
- *Physical Layer Node*
- Communicates with Rover Mega Controller (running on Arduino Mega) using custom STXETX protocol over UART connection. It handles communication in both directions.

`Protocol Hub Node`:
- Decodes & encodes STXETX protocol messages.
- Routes decoded messages to proper channels (ROS2 topics)
  
`Forward & Inverse Kinematics Node`:
- These nodes transform rover heading, i.e. speed and direction, to angular velocities of the wheels required for rover to have given heading (inverse kinematics) and *vice versa* (forward kinematics).

`Rover Controller Node`:
- Takes *target position* (*position setpoint*) and *current rover position* and periodically outputs *rover heading commands* to steer the rover from *current* to *target position*.

`Pose Estimator Node`:
- Implements algorithm for estimating current rover position from current position readings from *Marvelmind* indoor GPS system augumented with position feedback data derived from rover's (*Rover Mega Controller*) odometry feedback data.
- NOTE: Current verison uses only *Marvelmind* positional data (without integrating odometry feedback) with simple filtering and outlier rejection as basis for position estimate output, but the node was designed with (Extended) Kalman Filter algorithm in mind.

`Marvelmind Beacon Left/Right Node`:
- *Physical Layer Nodes*
- Official ROS2 nodes from *Marvelmind* repository
- Act as the interface between FRIPS C&C Stack and physical *Marvelmind* beacons. Communicate using UART via USB (Virtual COM).

`Marvelmind Position Fusion Node`:
- Since two *Marvelmind* beacons are used, one can only give position while two can give both position and orientation, this node is responsible for providing the rest of the FRIPS C&C Stack with information on rover's position and orientation based on positions of each beacon (left/right).
- This node has two modes of operation (see [Parameters Manual](/documentation/rover_launcher/Parameters.md)), controlled by `are_beacons_paired` switch in [`params.yaml`](/src/rover_launcher/config/params.yaml) depending if the beacons are paired manually (refer to the official *Marvelmind* documentation for more information) i.e. beacons broadcast, not their own position, but position and orientation of the beacon pair. Other mode of operation is manual pair position and orientation calculation based on each beacon's specific position.

## Further Upgrades

The system, as described in the previous chapter, is a part of the whole design. The system was designed to support autonomous navigation as well as obstacle avoidance, but due to time and quality restrictions, only a part of the system was implemented. The rest of the system will be implemented in the future.

![Illustration of the original (whole) FRIPS C&C Stack design](/documentation/assets/ROS_Infrastructure_TODO.png)

The figure above lays out the design of the original (whole) system. The nodes in red are not yet implemented, but are a part of the whole design that supports autonomous/intelligent functions.

Below is an overview of unimplemented nodes adn their tasks:

`Sonar Array Node`:
- The main task of this node is obstacle detection.
- Interfaces with existing *Arduino Zero/M0* on the rover, which is connected to the 12-element sonar array mounted on the sides of the rover.
- Provides the *Stack* with information on the environment, primarily obstacles in the rover's vicinity.

`Path Planning Node`:
- Node that has 2 primary functions; constructing an *image* of the environment and path planning
- Environment can be represented in any form e.g. 2D array which is the quantized map of the rover's enviroment
- Path planning can be implemented using any viable algorithm, depending how the environment is represented (*image*) e.g. *Potential Fields Method* combined with *A\* Path Search Algorithm*.
- The path is published to the `Waypoint Controller Node` for further processing.

`Waypoint Controller Node`:
- Node that receives a path from `Path Planning Node`, in any form used and defined by the mentioned node, and transforms it into a series of *3D points* which will be published down the Stack (to `Rover Controller Node`) as *target positions (position setpoints)*, one by one, until the rover reaches the end of the path. The third dimension is the height of the scissor lift, which should have a separate node designed and implemented.
- The node should have the ability to handle dynamic path i.e. a path changing in real time.  


