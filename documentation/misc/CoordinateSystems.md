# Coordinate Systems

Since the main objective of *FRIPS Command and Control ROS2 Stack* is to navigate the rover from point to point, **at least one** coordinate system must be defined.

**NOTE:** All coordinate systems defined are 3D beacuse rover can, theoretically, move in all three directions: 2D floor-level space and third dimension by controlling the height of scissor lift (which will be mounted on the rover).

## Defining the Coordinate Systems

There are three main coordinate systems already implicitly defined and used throughout the software:

1. Inverse Kinematics Node (`rover_inv_kinm_node`) has one task to perform: Transform rover heading command into wheel angular velocities. To be able to perform this task, the node has to define the reference forward direction of the rover along with positive angular displacement direction. By defining these directions, a 3D (right-handed) coordinate system can be constructed which will be refered to as `Rover Local Frame - RLF (Rover Local Coordinate System - RLCS)`.

2. Rover Controler Node (`rover_controller_node`) takes one point as input and steers the rover so it reaches the given point, all while the rover has access to its live position. A frame moving with rover, as defined in the previous point, would make calculations both complicated as well as resource intensive since every movement of the rover would mean that all obstacles and milestones, which are normally stationary relative to the environment/the Earth, would change their position in a frame that is fixed to the rover. By defining a stationary frame (relative to the Earth), all **stationary** obstacles and milestones have constant position while only rover has to recalculate its frame relative to this stationary frame, which greatly reduces CPU load removing unnecessary transformations. This frame, refered to as the `Rover Global Frame - RGF (Rover World Coordinate System - RWCS)`, is defined each runtime (more on that in a dedicated chapter) and represents a fixed point for positioning everything in rover's environment including the rover itself.

3. The main way of obtaining rover's position is using the *Techtile Marvelmind* indoor positioning system. The system measures time of flight of ultrasound waves from a moving beacon (mounted on the rover) to/from multiple fixed beacons to estimate moving beacon's (rover) position relative to any of the stationary beacons chosen as the reference beacon. The system defines its own coordinate system, refered to as the `Marvelmind Global Frame - MGF (Marvelmind World Coordinate System - MWCS)`, which is configured using official *Techtile Marvelmind* tools such as *Dashboard*. All *Marvelmind* positinal measurements are given relative to this frame of reference and need to be transformed before further use. This frame is also used to define the previous frame (more on that in a dedicated chapter).

These are three main coordinate systems/frames of reference used throughout the software. In following chapters their relationships are defined as formally as possible in an attempt to systematize their use throughout the software, including further upgrades, to prevent misuse and introduction of additional *ad hoc* coordinate systems due to lack of information. 

## Relationships between Coordinate Systems 


![Illustration of coordinate systems and their relationships](/documentation/assets/coordinate_systems_schematic.png)

Relationships between coordinate systems are illustrated in the figure above. The vectors between coordinate system origins illustrate hierarchy of frames and positional data flow through the system, which is presented in more detail in the figure below.

![Representation of data flow through the system](/documentation/assets/coordinate_systems_transform_pipeline.png)

Note that feedback is not yet implemented as of current version, but the system was developed with further upgrades in mind, including implementation of feedback data pathways, along with mechanisms that make this possible.

## Notes on using Coordinate System

To systematize the use of coordinate systems, here are some rules and guidelines for using mentioned coordinate systems:

`Rover Local Frame - RLF (Rover Local Coordinate System - RLCS)`:
  - A frame fixed to the rover, inconvenient for positioning and orientation.
  - Very convenient for rover kinematic calculations.
  - Should be only used sparingly on lowest layers of the *FRIPS C&C Stack* (Forward and Inverse Kinematics Nodes).
  
`Rover Global Frame - RGF (Rover World Coordinate System - RWCS)`:
  - A stationary frame, convenient for navigation, positioning, orientation, etc.
  - The main frame for positioning and orientation. Should be used as much as possible in place of other frames.
  - Defined at each runtime by publishing its position **relative to MWCS/MGF frame** to [`rover_world_origin_topic`](/documentation/rover_launcher/Sandboxing.md).

`Marvelmind Global Frame - MGF (Marvelmind World Coordinate System - MWCS)`:
  - A stationary frame, defined and used by *Techtile Marvelmind* indoor positioning system.
  - Configured independently by *Marvelmind Dashboard*.
  - All *Marvelmind* readings are given relative to this frame. They should be converted as soon as possible to a more suitable frame for calculation, such as **RWCS/RGF**.
  - Acts as the basis for defining **RWCS/RGF**.