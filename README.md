# PDM Project

Welcome to the PDM Project Group 38

*Contributors: Chiel de Dood (4915607), Alexandre Ferreira (6282598), Martijn Habers (5064767), and Jason Kim (6163246)*



## Table of Contents

- [Overview](#overview)
- [Packages](#packages)
  - [prm_navigation](#prm_navigation)
  - [sjtu_drone_bringup](#sjtu_drone_bringup)
  - [sjtu_drone_control](#sjtu_drone_control)
  - [sjtu_drone_description](#sjtu_drone_description)
  - [world_creator](#world_creator)
  - [filled_world.world](#filled_worldworld)

## Overview

This Project is structured to provide a framework for simulating and controlling drones within a 3D environment. It includes modules for path planning using Probabilistic Roadmaps (PRM), drone control logic, environment description, and world creation.

## Packages

### prm_navigation

This package is responsible for path planning using the Probabilistic Roadmap (PRM) algorithm with Dijkstra path finding. It takes an occupancy map as input and computes an optimal path for the drone to navigate through the environment.

#### Instructions to run prm_navigation node:

The `prm_navigation` package contains a node that, when active, subscribes to the `/goal_position` topic. 
The resulting `PoseArray` message is published to the `/drone_trajectory` topic.

Assuming that the workspace is built and sourced, you can run the node using the following command:

```bash
ros2 run prm_navigation prm_node


### sjtu_drone_bringup

This package contains the necessary configurations and launch files to initialize the drone model within the Gazebo simulation environment. It sets up the simulation parameters and ensures the drone is ready for operation.

### sjtu_drone_control

This package implements the control algorithms required for drone operation. It manages the drone's flight dynamics for stable and responsive control during simulation.

#### Instructions to run sjtu_drone_control

The `sjtu_drone_control` package contains a node with the same name that subscribes to the `/drone_trajectory` topic and navigates through the waypoints. When it runs out of waypoints, it publishes a string to the `/action_topic` to request new waypoints.

Assuming that the workspace is built and sourced, you can run the node using the following command:

```bash
ros2 launch sjtu_drone_control drone_control_launch.py

This launch file also starts the gazebo simulation and spawns the drone using the `sjtu_drone_bringup` package.

### sjtu_drone_description

This package provides the Unified Robot Description Format (URDF) files that define the drone's physical and visual properties. It also includes the world description, detailing the simulated environment in which the drone operates.

### world_creator

This package is designed to convert 3D world models into occupancy maps. These maps are essential for path planning algorithms like PRM, as they represent the navigable and obstructed areas within the environment.



### filled_world.world

This is the 3D world environment file used in the simulation. It defines the layout, obstacles, and other elements present in the simulated environment for a realistic scenario for drone operation.
