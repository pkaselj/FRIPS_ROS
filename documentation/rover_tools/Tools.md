# Tools

Using *sandbox* configuration, nodes and data can be emulated and faked for testing purposes. They can be emulated manually using ROS2 tools such as *rqt*.

The FRIPS C&C Stack comes with multiple tools in [`rover_tools`](/src/rover_tools/) package to automate and make node and data emulation simpler for the developer, some of which are:
- [`simulator`](/src/rover_tools/rover_tools/simulator_node.py) - emulates *Physical Layer* and provides visualization of rover position, heading and target position. Useful for testing and demonstrating higher-level algorithms (navigation, control, autonomy, etc.). 
- [`visualizer`](/src/rover_tools/rover_tools/visualizer_node.py) - similar to the `simulator` but does not provide data back to the FRIPS C&C Stack, instead it just visualizes received positional data (rover position & heading and target position). Useful for analyzing recorded live positional data replayed using `ros2 bag`.

![A schematic diagram explaining how tools interact with the FRIPS C&C Stack](/documentation/assets/tools_sandboxing.png)

## Launching and Using the Tools

Required for all tools:
- Source ROS2 underlay e.g. Linux: `source /opt/ros/humble/local_setup.sh` or Windows: `call C:\ros\humble\local_setup.bat`
- Source FRIPS C&C Stack overlay: `source install/setup.sh` or `call install\setp.bat`

`simulator`:
- launch FRIPS C&C Stack in *sandbox* configuration - `ros2 launch rover_launcher sandbox.launch.py`
- run `simulator_node` from `rover_tools` package - `ros2 run rover_tools simulator_node`
- Interact with the system by publishing messages to topics, preferrably using ROS2 `RQT` tool.

`visualizer`:
- run `visualizer_node` from `rover_tools` package - `ros2 run rover_tools visualizer_node`
- Publish replay saved for visualization, preferrably using `ros2 bag` i.e. `ros2 bag play saved_data/sample1/sample1.db3`


## Examples

The tools use `PyGame` for UI. An example screenshot of the FRIPS Visualizer can be seen on figure below.

![Screenshot of Visualizer tool in action](/documentation/assets/rover_tools_example.png)

Following information can be read from the tools UI:
- *Current position* - Current position of the rover relative to *RWCS/RGF* frame
- *Target position* - Setpoint position for the rover telative to *RWCS/RGF* frame
- *Commanded heading* - Heading that *Rover Controller Node* commanded to the rover so it can reach the target position from its current position.
- *Rover forward direction* - Direction of rover's forward axis i.e. *x-axis* of the *RLCS/RLF* frame relative to *RWCS/RGF* frame

The coordinates used by tools are relative to *RWCS/RGF* frame. The center (0,0) is at the midpoint of the UI. Positive directions for *x* (horizontal) and *y* (vertical) axes are right and down, respectively.