# IndusNav-AMR: Autonomous Mobile Robot Navigation and Docking System

An Autonomous Mobile Robot (AMR) simulation stack for ROS 2 Jazzy and Gazebo, featuring a full Nav2 integration, MPPI control, and a custom state-machine for battery-aware autonomous docking.

## Overview

IndusNav-AMR is a simulation reference stack demonstrating advanced warehouse robotics behaviors in ROS 2. It goes beyond standard SLAM and point-to-point navigation by integrating a fully functional **battery lifecycle and auto-docking system**. 

The project showcases how an AMR balances active work (waypoint patrolling) with self-preservation (navigating to a charging station before battery depletion) using custom Python logic and ROS 2 Action Servers.


## Core Capabilities

This workspace packages a complete continuous AMR workflow:

| Feature | Description |
| :--- | :--- |
| **Simulation Environment** | Gazebo industrial warehouse world with a URDF/Xacro differential-drive robot description. |
| **Advanced Navigation** | Nav2 stack utilizing the **MPPI controller** (configured for smooth, kinematically feasible trajectories) and SMAC planner. |
| **Continuous Operations** | Waypoint patrol (`patrol_node`) that runs missions in a continuous loop until interrupted. |
| **Power Simulation** | Custom battery node simulating active drain and charging rates based on the robot's physical state. |
| **Autonomous Docking** | Integrates the `opennav_docking` server with a custom Python state machine (`auto_dock_manager`) to gracefully preempt missions, safely approach the dock, and resume work when charged. |

## Technology Stack

| Component | Technology |
| :--- | :--- |
| **OS Target** | Ubuntu 24.04 LTS |
| **ROS Distribution** | ROS 2 Jazzy Jalisco |
| **Simulator** | Gazebo |
| **Robot Model** | Differential Drive AMR (URDF) + Lidar & Camera sensors |
| **Navigation Server** | Nav2 with `nav2_mppi_controller` |
| **Docking Server** | `opennav_docking` Action Server |
| **Logic & Control** | Python3 (`rclpy`), Multi-threading, Action Clients |

## Repository Layout

```text
navigation_pkg/
├── config/
│   └── mapper_params_online_async.yaml
│   └── nav2_params.yaml         
├── launch/
│   └── docking.launch.py    
    └── navigation.launch.py
    └── rsp.launch.py
    └── simulation.launch.py
    └── slam.launch.py
├── maps/
│   ├── industrial.yaml           
│   └── industrial.pgm        
├── navigation_pkg/
│   ├── auto_dock_manager.py      
│   ├── battery_simulator.py      
│   ├── dock_pose_broadcaster.py  
│   └── patrol_node.py  
    └── person_detector.py
    └── webcam_publisher.py
├── rviz/
│   └── rviz_config.rviz  
├── urdf/
│   └── patrol_robot.urdf.xacro          
└── package.xml                  
