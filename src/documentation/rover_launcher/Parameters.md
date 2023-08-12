# FRIPS_ROS Project Parameters Overview

The purpose of this document is to list and explain all parameters available in [`params.yaml`](src\rover_launcher\config\params.yaml) file.

## Parameters by Node

---
---

### marvelmind_ros2_left_beacon

ROS2 node that interfaces with Marvelmind Beacon via USB. There are two beacons on rover, __left__ and right, so that both position and orientation can be determined.

\* See official Marvelmind ROS2 package for more parameters

|Parameter Key|Type|Unit|Default Value|Comment|
|---|---|---|---|---|
|hedgehog_pos_topic|`String`|-|"/unused/hedgehog_pos_topic"| UNUSED |
|hedgehog_pos_angle_topic|`String`|-|"/position_measurement/left"|Topic for receiving __Marvelmind Beacon Left__ position or, if paired, pair position and orientation.|
|marvelmind_publish_rate_in_hz|`Int`|Hz|30||
|marvelmind_tty_baudrate|`Int`|Baud|9600||
|marvelmind_tty_filename|`String`|-|"/dev/ttyACM0"|__Marvelmind Beacon Left__ USB port name (On Windows use COMx i.e. COM3 or "\\\\.\\COM3").|

---
---

### marvelmind_ros2_right_beacon

ROS2 node that interfaces with Marvelmind Beacon via USB. There are two beacons on rover, left and __right__, so that both position and orientation can be determined.


\* See official Marvelmind ROS2 package for more parameters

|Parameter Key|Type|Unit|Default Value|Comment|
|---|---|---|---|---|
|hedgehog_pos_topic|`String`|-|"/unused/hedgehog_pos_topic"| UNUSED |
|hedgehog_pos_angle_topic|`String`|-|"/position_measurement/right"|Topic for receiving __Marvelmind Beacon Right__ position or, if paired, pair position and orientation.|
|marvelmind_publish_rate_in_hz|`Int`|Hz|30||
|marvelmind_tty_baudrate|`Int`|Baud|9600||
|marvelmind_tty_filename|`String`|-|"/dev/ttyACM1"|__Marvelmind Beacon Right__ USB port name (On Windows use COMx i.e. COM4 or "\\\\.\\COM4").|

---
---

### rover_controller_node

Node that takes current position (and orientation in `Rover Global Frame - RGF (Rover World Coordinate System - RWCS)`) from `position_topic` and a point (in the same frame) where rover wants to be, from `setpoint_topic`, and produces heading information (rover velocity in `m/s`, heading angle relative to current rover orientation - which is actually just heading in `Rover Local Frame - RLF (Rover Local Coordinate System - RLCS)` and duration in `ms` which will get rover from current to set position given velocity and heading). 

|Parameter Key|Type|Unit|Default Value|Comment|
|---|---|---|---|---|
|command_topic|`String`|-|/relative_heading_setpoint|Topic to which the node publishes `RoverHeading` messages which contain rover velocity (m/s), heading relative to current orientation (rad) and duration (ms).|
|controller_commands_topic|`String`|-|/controller_commands|Topic for controlling the node. Receives "START"/"STOP" commands.|
|position_topic|`String`|-|/position_estimate| Topic from which the node gets current position estimate.|
|setpoint_topic|`String`|-|/position_setpoint| Topic form which the node gets current setpoint position.|
|publish_rate_ms|`Int`|ms|250|Publish rate for heading (`command_topic`).|
|max_speed_magnitude|`Float`|m/s|0.07| Max. speed to which the rover is limited to prevent jerky behavior (see `command_topic`).|

---
---

### rover_fwd_kinm_node

Node that takes current wheel velocities (RPM,RPS), from feedback odometry, and converts them to rover speed and orientation given a kinematic model of the rover. Used to provide rover heading, from feedback odometry, to position estimation algorithms.

|Parameter Key|Type|Unit|Default Value|Comment|
|---|---|---|---|---|
|odometry_input_topic|`String`|-|/odometry| Odometry feedback input.|
|odometry_input_qos|`Int`|-|10|See ROS2 topic QoS value information. Basically queue size.|
|relative_heading_output_topic|`String`|-|/relative_heading_estimate_from_odometry| Rover kinematics estimate output from odometry feedback and given rover kinematic model.|
|relative_heading_output_qos|`Int`|-|10|
|wheel_to_body_center_distance_m|`Float`|m|0.200| Distance from the rover's center to the center of the wheel.|
|wheel_radius_m|`Float`|m|0.075||

---
---

### rover_interface_node

Node that bi-directionally communicates with Rover Controller on Arduino Mega over Serial UART. All messages are encoded using custom STXETX Protocol.

All inbound messages (messages received from Rover Controller on Arduino Mega and intended for higher layers) are converted from binary STXETX packets to custom ROS2 message representation of STXETX packets and are published to `received_msg_broadcast` topic for futher decoding and routing. All outbound messages (received on topic `transmit_request` from higher layers and intended for Rover Controller on Arduino Mega) are converted from custom ROS2 STXETX message representation to binary STXETX packet and sent to Rover Controller on Arduino Mega via UART.

This node is responsible for encoding and decoding STXETX packets into ROS2 STXETX Frames (custom simplified object representation of STXETX packet).

|Parameter Key|Type|Unit|Default Value|Comment|
|---|---|---|---|---|
|port_name|`String`|-|COM3| Serial port which is used to communicate with Rover Controller on Arduino Mega.|
|baud_rate|`Int`|Baud|9600|
|received_msg_broadcast|`String`|-|/rover_receiver| Topic on which all messages received from Rover Mega Controller are broadcast in form of custom ROS2 STXETX message representation.|
|transmit_request|`String`|-|/rover_transmitter| Topic from which all messages are serialized and sent to Rover Mega Controller via UART. |
|received_msg_broadcast_qos|`Int`|-|10|
|transmit_request_qos|`Int`|-|10|

---
---

### rover_inv_kinm_node

Node that takes rover velocity (`m/s`), relative heading (heading in `Rover Local Frame - RLF (Rover Local Coordinate System - RLCS)`) and converts them to angular velocities of each of three wheels using inverse kinematics model of the rover.


|Parameter Key|Type|Unit|Default Value|Comment|
|---|---|---|---|---|
|wheel_to_body_center_distance_m|`Float`|m|0.225|Distance from the rover's center to the center of the wheel.|
|wheel_radius_m|`Float`|m|0.075|
|heading_input_topic|`String`|-|/relative_heading_setpoint| Input topic for inverse kinematics.|
|heading_input_qos|`Int`|-|10|
|command_output_topic|`String`|-|/command| Inverse kinematics output topic (wheel angular velocities/RPM,RPS).|
|command_output_qos|`Int`|-|10|

---
---

### rover_pose_estimator_node

This node runs estimation algorithm which takes (prefiltered) position input from two sources:
1. Techtile Marvelmind beacon measurement (via `marvelmind_position_fusion_node`) 
2. Odometry feedback data (form Rover Mega Controller) transformed to relative heading data (via `rover_fwd_kinm_node`). __NOTE: Heading data must be integrated over time to get total displacement so that position can be calculated from displacement and last known position.__

And calculates the most probable current position. The node was designed with (Extended) Kalman Filter in mind, although, due to hard deadlines, only a simple filtering of __Techtile Marvelmind beacon measurement__ is used as estimated position. TODO.

|Parameter Key|Type|Unit|Default Value|Comment|
|---|---|---|---|---|
|rover_world_origin_topic|`String`|-|/rover_world_origin_topic|Topic that takes coordinate in `Marvelmind Global Frame - MGF (Marvelmind World Coordinate System - MWCS)`, which is a coordinate system in which Marvelmind system functions, which is then set as the origin for `Rover Global Frame - RGF (Rover World Coordinate System - RWCS)` and all subsequent measurements are transformed into `RGF/RWCS` frame.|
|position_input_topic|`String`|-|/position_measurement|Input topic for raw (or, preferably, prefiltered) data from Marvelmind beacons (via `marvelmind_position_fusion_node`). Coordinates are expected to be in `MGF/MWCS` frame.|
|position_output_topic|`String`|-|/position_estimate|Output topic for position estimate (in `RGF/RWCS` frame).|
|relative_heading_input_topic|`String`|-|/relative_heading_estimate_from_odometry| Input topic for odometry feedback data which was transformed to relative heading data by `rover_fwd_kinm_node`.|
|broadcast_rate_ms|`Int`|ms| 10| The rate at which the position estimate is calculated and broadcast. Input data is cached and saved during this interval.|

---
---

### rover_protocol_hub_node

Node that receives STXETX messages (ROS representation of STXETX packets) from `rover_interface_node`, which are deserialized STXETX packets received from Rover Mega Controller (Rover Controller on Arduino Mega) intended for higher layers. It futher decodes each message, including message's payload (decode callback based on STXETX packet's `msg_type` field), and broadcasts (routes) it on its own topic.

It also received messages from higher layers intended for Rover Mega Controller and converts them to STXETX Frames (custom simplified ROS2 object representation of STXETX packet) and relays them to the `rover_interface_node` for serialization and sending to Rover Mega Controller. __Note that by definition, STXETX protocol is Little Endian__.

This node is intended as translation layer between higher layer objects and STXETX Frames (which are further encoded into STXETX packets by `rover_interface_node`).

|Parameter Key|Type|Unit|Default Value|Comment|
|---|---|---|---|---|
|rover_msg_subscriber_topic|`String`|-|/rover_receiver| Input topic from which the node receives STXETX Frame from `rover_interface_node` (indirectly, Rover Mega Controller) and routes message for further decoding and broadcasting. Single point fo entry for incoming messages from `rover_interface_node`/Rover Mega Controller. |
|rover_msg_subscriber_qos|`Int`|-|10|
|rover_msg_publisher_topic|`String`|-|/rover_transmitter| Output topic to which all STXETX Frames outbound for Rover Mega Controller are broadcast to be serialized by `rover_interface_node`.| 
|rover_msg_publisher_qos|`Int`|-|10|
|odometry_publisher_topic|`String`|-|/odometry|Output topic on which all inbound odometry messages (which are obtained from Rover Mega Controller/`rover_interface_node`) are published after decoding. |
|odometry_publisher_qos|`Int`|-|10|
|command_listener_topic|`String`|-|/command| Input topic that takes all `command` messages (which contain setpoint angular velocities/RPM,RPS for each wheel), encodes message to STXETX Frame and relays it to the `rover_interface_node` via `rover_msg_publisher_topic`.
|command_listener_qos|`Int`|-|10|

---
---

### marvelmind_position_fusion_node

This node acts as an interface between Marvelmind beacons and pose estimation algorithm in `rover_pose_estimator_node`. It acts as the source of filtered pose measurement data.

In case the two Marvelmind beacons are paired, each of them broadcasts pair position (center position between two nodes or as is setup in Marvelmind Dashboard) so this node acts as a simple filter for the center position. To work in this mode, parameter `are_beacons_paired` must be set to `True`.

In case the `are_beacons_paired` is set to `False`, this node takes position of each beacon (beacons do not broadcast their position exclusively) and calculates position of beacon pairs (as if they were paired), manually.

This data is further used as input to the pose estimation algorithm running in `rover_pose_estimator_node` node.

|Parameter Key|Type|Unit|Default Value|Comment|
|---|---|---|---|---|
|position_measurement_left_topic|`String`|-|/position_measurement/left|Position input topic from left Marvelmind beacon.| 
|position_measurement_left_qos|`Int`|-|10|
|position_measurement_right_topic|`String`|-|/position_measurement/right|Position input topic from right Marvelmind beacon|
|position_measurement_right_qos|`Int`|-|10|
|fused_position_topic|`String`|-|/position_measurement|Output topic where position of the beacon pair (fused position) is broadcast.| 
|fused_position_qos|`Int`|-|10|
|fused_position_broadcast_rate_hz|`Float`|Hz|50.0| Rate at which the fused/pair position is broadcast on `fused_position_topic`.|
|fusion_exponential_averaging_alpha|`Float`|- ([0.0, 1.0])|0.1| Old measurement coefficient for exponential averaging filter for fused position. Applied only in case the beacons are paired, else individual positions are stored to buffers of size `position_buffer_size` and are averaged (as in filtering) before calculating center position manually from averaged/filtered values| 
|center_distance_from_left_node_percent|`Float`|% (0.0-1.0)|0.5| Distance of the ceter of the pair from the left Marvelmind beacon, expressed in percentages of the pair length/distance between two beacons.|
|pair_distance_m|`Float`|m|2.0|Pair length/distance between two beacons expressed in meters.|
|are_beacons_paired|`Boolean`|-|False|Switch to change node's behavior.|
|position_buffer_size|`Int`|-|2|In case pair center position is calculated manually, positions of each end of pair (left and right beacons) are first filtered by averaging last `position_buffer_size` samples and then the center position is calculated from the filtered data to reduce jitter and noise.|
|left_beacon_address|`Int`|-|7| Marvelmind Beacon Left address.|
|right_beacon_address|`Int`|-|8| Marvelmind Beacon Right address.

---
---