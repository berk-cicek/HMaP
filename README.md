# H-MaP: Iterative and Hybrid Sequential Manipulation Planner
## Overview
<p align="center">
  <img src="https://github.com/berk-cicek/HMaP/blob/main/misc/Diagram.svg" alt="HMaP"/>
</p>

This study introduces the Hybrid Sequential Manipulation Planner (H-MaP), a novel approach that iteratively does motion planning using contact points and waypoints for complex sequential manipulation tasks in robotics. Combining optimization-based methods for generalizability and sampling-based methods for robustness, H-MaP enhances manipulation planning through active contact mode switches and enables interactions with auxiliary objects and tools. This framework, validated by a series of diverse physical manipulation tasks and real-robot experiments, offers a scalable and adaptable solution for complex real-world applications in robotic manipulation.
The key idea is the iterative computation of the contact points among waypoints.This iterative approach enhances the robustness of the solver.Waypoints discretize the problem into smaller steps, and contact points guide the motion planner for switching contact modes.Contact modes facilitate the dynamic adjustment of the robot's interaction with objects during motion planning, guiding the solver on how and where the manipulator should engage or disengage with objects.Combined with an optimization-based planner, waypoint and contact point information enable the solver to accomplish complex sequential manipulation tasks.

https://sites.google.com/view/h-map/

## Installation
### Requirments
Assumes a standard Ubuntu 20.04 (or 18.04) machine.
### Quick Start
``` 
git clone https://github.com/berk-cicek/HMaP
```
#### Build the RAI submodule.
```
pip install robotic
pip show robotic
```

## Example Usage
<div align="center">
  <img src="https://github.com/berk-cicek/HMaP/blob/main/misc/bolt.gif" width="250" height="250" /> 
  <img src="https://github.com/berk-cicek/HMaP/blob/main/misc/tunnel.gif" width="250" height="250" />
</div>

<p align="center">
  <img src="https://github.com/berk-cicek/HMaP/blob/main/misc/Experiments.png" alt="HMaP1"/>
</p>

<p align="center">
  <img src="https://github.com/berk-cicek/HMaP/blob/main/misc/Realrobot.JPG" alt="HMaP2"/>
</p>
