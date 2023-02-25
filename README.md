# Binary-occupancy-grid

**Objective:**
The objective of the project was to map an arena for a mobile robot using a binary occupancy  grid and find the best path between two points using probabilistic route mapping.

**Introduction:**
A mobile robot is an autonomous system or vehicle that can navigate or move in a given 
environment. The three fundamental questions that are important and need to be solved for 
creating an autonomous mobile robot are- “Where am I?” (Localization problem), “Where am I 
going?” (Target or goal), and “How do I get there?” (Navigation). In this project, we worked on 
one of the problems in localization which is mapping by creating a binary occupancy grid for 
the physical map. Then we took graphical input to determine the starting and ending position 
and finally to generate the best path for navigation we used probabilistic route mapping. 

A binary occupancy grid uses true values to represent the occupied workspace (obstacles)
and false values to represent the free workspace. This grid shows where obstacles are and 
whether a robot can move through that space. They are used to represent a robot workspace 
as a discrete grid and are used in robotics algorithms such as path planning. Information about 
the environment can be collected from sensors in real time or loaded from prior knowledge. 
Laser range finders, bump sensors, cameras, and depth sensors are commonly used to find 
obstacles in the robot’s environment.

Probabilistic route planning is a motion planning algorithm in robotics, which solves the 
problem of determining a path between a starting configuration of the robot and a goal 
configuration while avoiding collisions. It is a network graph of possible paths in a given map 
based on free and occupied spaces. Random nodes are generated and connections are created 
between these nodes based on the PRM algorithm parameters. Nodes are connected based 
on the obstacle locations specified in Map, and on the specified Connection Distance. The PRM 
algorithm uses the network of connected nodes to find an obstacle-free path from a start to 
an end location. To plan a path through an environment effectively, the number of 
nodes and Connection Distance properties are tuned. The key to tuning them is to not take 
too many nodes and really short connection distance since it will increase complexity and 
computation time as well as to not take too less nodes and really large connection distance 
since it will lead to no path being found between the two points or the path found won’t be as 
smooth.

**MATLAB Add-Ons used:**
1) Mobile robotics simulation Toolbox
2) Robotics System Toolbox
3) Symbolic math Toolbox
