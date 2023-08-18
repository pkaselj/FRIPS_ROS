# Tools

Using *sandbox* configuration, nodes and data can be emulated and faked for testing purposes. They can be emulated manually using ROS2 tools such as *rqt*.

The FRIPS C&C Stack comes with multiple tools in [`rover_tools`](/src/rover_tools/) package to automate and make node and data emulation simpler for the developer, some of which are:
- [`simulator`](/src/rover_tools/rover_tools/simulator_node.py) - emulates *Physical Layer* and provides visualization of rover position, heading and target position. Useful for testing and demonstrating higher-level algorithms (navigation, control, autonomy, etc.). 
- [`visualizer`](/src/rover_tools/rover_tools/visualizer_node.py) - similar to the `simulator` but does not provide data back to the FRIPS C&C Stack, instead it just visualizes received positional data (rover position & heading and target position). Useful for analyzing recorded live positional data replayed using `ros2 bag`.

![A schematic diagram explaining how tools interact with the FRIPS C&C Stack](/documentation/assets/tools_sandboxing.png)