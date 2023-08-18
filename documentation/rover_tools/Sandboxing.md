# Sandboxing

*For general information on FRIPS C&C Stack, please refer to the [FRIPS C&C Stack Overview Document](/documentation/general/FRIPS_C%26C_StackOverview.md).*

---

![Software Development Cycle](/documentation/assets/software_development_cycle.png)

The figure above illustrates the optimal cycle of a software (or any product) development process. The illustrated cycle, in ideal conditions, gives best results in terms of software robustness, performance and maintainability.

The concept of nodes in ROS2 naturally forces a developer to design isolated, abstracted and focused tasks that run independently or interdependently with other tasks. ROS2 nodes are, in simplified terms, containers that abstract certain apects of system, such as communications, timers, configuration, etc. For more information refer to the dedicated chapter for ROS2.

ROS2 effectively forces the developer to write *unit-testable* pieces of software, but the problem arises when system's interaction with itself (node/task interoperability) or with its environment (interaction with hardware) needs to be tested and evaluated. The crux of the problem is handling nodes of the *Physical Layer* i.e. nodes that directly interact with environment/hardware, such as `Rover Interface Node` and `Marvelmind Beacon Left/Right Nodes`.

![Schematic representation of the divide between Physical Layer and higher level layers](/documentation/assets/ROS_LayerBoundary.png)

To overcome this obstacle, a mechanism for sandboxing the FRIPS C&C Stack was introduced. The main task of the FRIPS C&C Stack is to position, orient and navigate the rover. To accomplish this task, the FRIPS C&C Stack has to obtain data from real physical hardware, such as *Marvelmind* beacons and *Rover Mega Controller*.

Testing high-level algorithms (like navigation, control or intelligent functions) on live hardware is complicated, carries risks of faulty algorithms causing damage to the rover and its components and reduces the ability of the developer to find edge cases by not allowing him to control the testing environment.

To provide the developer with proper facilities and mechanisms for testing algorithms in isolation i.e. without hardware, the FRIPS C&C Stack can be launched in **Sandbox** configuration; the [`sandbox.launch.py`](/src/rover_launcher/launch/sandbox.launch.py) launch file starts all nodes except *Physical Layer* Nodes (refer to [FRIPS C&C Stack Overview](/documentation/general/FRIPS_C%26C_StackOverview.md) for more information). Since ROS2 abstracts communication and node interaction by using the concept of *topics*, it allows the programmer to develop nodes that emulate real *Physical Layer* nodes and feed custom data to algorithms.

The FRIPS C&C Stack comes with multiple tools in [`rover_tools`](/src/rover_tools/) package which are described in the dedicated chapter ([Tools](/documentation/rover_tools/Tools.md)).

![Illustration of the underlaying principle behind sandboxing](/documentation/assets/ROS_SandboxPrinciple.png)