# Sandboxing

The main tasks of Rover Mega Controller software (Rover Controller software running on Arduino Mega) are:

1. Listen for commands (RPM/RPS setpoints for wheels) from FRIPS C&C Stack.
2. Control rotational velocities of all wheels (RPM/RPS) using PI(D) controllers.
3. Provide feedback on instantaneous wheel angular velocities (RPM/RPS) to FRIPS C&C Stack.

The FRIPS C&C (Command & Control) Stack is a software, consisting of a group of ROS2 nodes, whose purpose is to:

1. Transform user input (Location) to rover commands for Rover Mega Controller
2. Control, Manage and Supervise all processes require to perform tasks laid out by the point above.

The following graph shows all nodes in FRIPS C&C Stack:

