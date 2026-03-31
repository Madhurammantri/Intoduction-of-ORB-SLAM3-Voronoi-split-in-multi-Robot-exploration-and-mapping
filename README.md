# Intoduction-of-ORB-SLAM3-Voronoi-split-in-multi-Robot-exploration-and-mapping

ROS 2 multi-robot exploration thesis project built on David Dudas’s original `multi_robot_navigation` work, extended with merged-map frontier exploration, Voronoi-based coordination, goal claiming, TF-stability-gated stopping, and optional ORB-SLAM3 + EKF fusion for improved localization.

# Multi-Robot Exploration and Mapping in ROS 2  
### Thesis Repository: ORB-SLAM3-Enhanced Multi-Robot Exploration with Centralized Map Merging

This repository contains the implementation and thesis-related extensions developed for my master’s thesis on **multi-robot indoor exploration and mapping in ROS 2**. The work focuses on a **merged occupancy-grid exploration pipeline** with **frontier-based exploration**, **Voronoi-partitioned coordination**, **goal claiming**, **TF-stability-gated stopping**, and **optional ORB-SLAM3 + EKF fusion** for improved localization.

## Overview

Autonomous indoor exploration with multiple robots is challenging because of localization drift, repeated exploration of the same areas, TF instability, and coordination conflicts between robots. This repository presents a ROS 2-based simulation framework in which each robot navigates and maps in its own namespace, while a **merged occupancy grid** is used as the common shared representation for frontier extraction and coordinated exploration.

The system builds on established ROS 2 components such as:

- **Nav2** for goal-oriented navigation
- **SLAM Toolbox** for online 2D LiDAR mapping
- **robot_localization EKF** for multi-sensor fusion
- **ORB-SLAM3** as an optional visual-inertial odometry source
- **Merged-map frontier exploration** for global reasoning across robots

## Main Contributions

This repository extends a baseline multi-robot navigation framework with thesis-specific modifications, including:

- **Merged occupancy-grid frontier exploration**
- **Voronoi-based region biasing for robot separation**
- **Goal claiming with exclusion radius and timeout**
- **TF-stability-gated stopping condition**
- **Optional ORB-SLAM3 visual-inertial odometry fused into EKF**
- **Simulation-based comparison of ORB vs. non-ORB exploration behavior**

## System Architecture

The overall exploration pipeline is based on the following idea:

1. Each robot performs mapping and navigation within its own namespace.
2. Local map information is merged into a **shared global occupancy grid**.
3. Frontiers are extracted from the merged map.
4. Candidate exploration goals are filtered using:
   - **Voronoi ownership**
   - **Goal claiming constraints**
5. Goals are executed through **Nav2**.
6. Exploration stops only when:
   - frontiers remain empty for a sustained duration, and
   - TFs remain stable for a required time window.

## Thesis Context

**Title:**  
*A Simulated Demonstration of a Multi-Robot Swarm Using ORB-SLAM in ROS2 with Centralized Map Merging to Reduce Drift: Introducing ORB-SLAM3 and Voronoi-Partitioned Frontier Exploration for Multi-Robot Exploration and Mapping on a Merged Occupancy Grid*

**Author:** Madhuram Mantri  
**Program:** M.Eng. Engineering and Sustainable Technology Management – Industry 4.0 Robotics, Automation & 3D Manufacturing  
**Institution:** SRH Berlin University of Applied Sciences  
**Supervisors:** Prof. Lukusz Rojek, Prof. Dr. Hartmann

## Research Focus

The objective of this work is to design and evaluate a ROS 2 multi-robot exploration system that reduces redundant exploration while maintaining robust navigation and localization. The thesis specifically studies:

- frontier-based exploration on a merged occupancy grid
- coordination using Voronoi partitioning and goal claiming
- the effect of navigation design on exploration efficiency
- the impact of EKF fusion with optional ORB-SLAM3 odometry on pose consistency and exploration performance

## Key Features

- ROS 2 multi-robot simulation workflow
- Centralized merged-map exploration
- Frontier clustering and goal generation
- Voronoi-based coordination
- Goal-claim conflict reduction
- Nav2-based repeated goal execution
- SLAM Toolbox integration
- EKF-based sensor fusion
- Optional ORB-SLAM3 integration
- Comparative exploration experiments across environments

## Repository Origin and Attribution

This repository builds upon the original work by **David Dudas**:

**Original repository:**  
`multi_robot_navigation`  
https://github.com/dudasdavid/multi_robot_navigation

I gratefully acknowledge the original repository as the baseline on top of which this thesis work was developed. My thesis extends that foundation with additional coordination, stopping logic, merged-map exploration changes, and optional ORB-SLAM3/EKF fusion. The thesis acknowledgement also explicitly credits Prof. David Dudas for allowing continuation of the work.

## AI Assistance Disclosure

Parts of the code, documentation, refactoring, and repository presentation were developed with **AI-assisted support**.  
However, the **core thesis concept, system design, integration choices, methodology, experiments, and validation are my own**.

## License

This repository is distributed under the **Apache License 2.0**.

Because this work builds upon an Apache-licensed original repository, this repository preserves compatible licensing and attribution.

See the [LICENSE](LICENSE) file for details.
