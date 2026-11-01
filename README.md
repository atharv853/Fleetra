# Fleetra 🚀

> **A Networked Electronic Fleet Tasking (NEFT) system for multi-agent autonomous warehouse orchestration.**

[![ROS 2](https://img.shields.io/badge/ROS%202-Foxy-blue.svg)](https://docs.ros.org/en/foxy/)
[![Navigation](https://img.shields.io/badge/Nav2-Navigation%202-orange.svg)](https://navigation.ros.org/)
[![Simulation](https://img.shields.io/badge/Gazebo-11-yellow.svg)](http://gazebosim.org/)
[![DDS](https://img.shields.io/badge/Middleware-CycloneDDS-brightgreen.svg)](https://cyclonedds.io/)
[![Docker](https://img.shields.io/badge/Container-Docker-2496ED.svg)](https://www.docker.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

---

## Overview

**Fleetra** provides an end-to-end framework for coordinating multiple Autonomous Mobile Robots (AMRs) operating cooperatively inside a dense warehouse environment. AMRs navigate to target storage racks ("pallets"), elevate them using an actuated lift mechanism, transport them along warehouse aisles to designated drop-off stations, and lower them into position without human intervention or agent deadlocks.

The system integrates **ROS 2 Navigation (Nav2)**, **Gazebo 11 physics simulation**, **CycloneDDS loopback networking**, and an intelligent **turn-taking task sequencer**.

---

## Key Features

- **Multi-Agent Coordination & Sequencing**:
  - Centralized task sequencer (`multi-robot.py`) handles multi-stage jobs (approach, pick, aisle extraction, transit, drop-off).
  - Aisle staging and turn-taking rules avoid bottlenecks and prevent collisions in narrow warehouse corridors.
  - Automatic goal retry with configurable timeouts.
- **Single-Command Automated Launch**:
  - One-click launch (`./run_demo.sh`) starts Docker container, launches Gazebo simulation, starts dual Nav2 stacks, and opens RViz instances.
  - Fully automated localization and startup sequence—no manual "2D Pose Estimate" clicking in RViz required.
- **Dynamic LiDAR Footprint Masking**:
  - `rack_scan_filter.py` applies a real-time bounding box filter on laser scans (`/robotN/scan` → `/robotN/scan_filtered`).
  - Ignores returns from the legs of a carried rack so the robot doesn't perceive its own payload as an obstacle.
- **Automated Pose Estimation & Scan Matching**:
  - `set_initial_poses.py` reads ground-truth positions from Gazebo, correlates 2D LiDAR rays against the occupancy grid map, and continuously publishes initial poses until AMCL converges.
- **High-Density DDS Tuning**:
  - Custom CycloneDDS unicast loopback profile (`scripts/cyclonedds_localhost.xml`) ensures reliable discovery across 80+ ROS nodes without packet drops or external Wi-Fi dependencies.
- **Visual Robot Identification**:
  - Custom Gazebo robot spawner dynamically generates red 7-segment numeral plates ("1" and "2") onto each robot's lift plate for clear visual distinction.

---

## System Architecture

```text
                               +-----------------------------+
                               |     Fleetra Sequencer       |
                               | (multi-robot.py / Action)   |
                               +--------------+--------------+
                                              |
                   +--------------------------+--------------------------+
                   |                                                     |
                   v                                                     v
        +---------------------+                               +---------------------+
        |       robot1        |                               |       robot2        |
        | - Nav2 Stack        |                               | - Nav2 Stack        |
        | - AMCL Localization |                               | - AMCL Localization |
        | - Costmap / Planner |                               | - Costmap / Planner |
        +----------+----------+                               +----------+----------+
                   |                                                     |
                   +--------------------------+--------------------------+
                                              |
                                              v
                               +-----------------------------+
                               |   CycloneDDS (Loopback)     |
                               +--------------+--------------+
                                              |
                                              v
                               +-----------------------------+
                               |    Gazebo 11 Simulation     |
                               |   (AWS Warehouse World)     |
                               +-----------------------------+
```

---

## Directory Structure

```text
Fleetra/
├── CustomRobots/
│   └── amazon_robot/            # ROS 2 packages (bringup, controller, description, gazebo, etc.)
│       ├── amazon_robot_bringup/
│       ├── amazon_robot_controller/
│       ├── amazon_robot_description/
│       ├── amazon_robot_gazebo/
│       ├── amazon_robot_msg/
│       ├── amazon_robot_rviz_plugins/
│       ├── aws-robomaker-small-warehouse-world/
│       └── exercise/            # Multi-robot coordination script (multi-robot.py)
├── docker/
│   └── Dockerfile               # Reproducible multi-stage Docker build for ROS 2 Foxy & Nav2
├── logs/                        # Execution and launch logs (gitignored)
├── params/                      # Nav2 tuned configuration parameters and warehouse occupancy maps
│   ├── map_3beds/               # 2D occupancy grid map
│   ├── nav2_multirobot_params_1_with_control.yaml
│   └── nav2_multirobot_params_2_with_control.yaml
├── scripts/
│   ├── activate_nav2.py         # Nav2 lifecycle manager state machine transitioner
│   ├── count_msgs.py            # Diagnostic message counter for topics
│   ├── cyclonedds_localhost.xml # CycloneDDS loopback unicast configuration
│   ├── fastdds_udp_only.xml     # Alternative FastDDS configuration
│   ├── rack_scan_filter.py      # Real-time LiDAR box filter for carried racks
│   └── set_initial_poses.py     # Automated Gazebo ground-truth AMCL initializer
├── DEMO_GUIDE.md                # In-depth technical guide, debugging notes, and problem history
├── how_to_run.md                # Operator quick-start and demonstration guide
├── LICENSE                      # MIT License
├── README.md                    # Project documentation
├── run_demo.sh                  # Master one-command execution script
└── stop_demo.sh                 # Clean teardown and process termination script
```

---

## Quick Start

### Prerequisites

- Linux (Ubuntu 20.04+, Arch Linux, Debian, Fedora)
- [Docker](https://docs.docker.com/get-docker/) installed and current user added to the `docker` group (`sudo usermod -aG docker $USER`)
- X11 or Wayland display server with GPU/iGPU acceleration support

### 1. Run the Demo

Run the automated orchestrator from the project root:

```bash
./run_demo.sh
```

The script will automatically:
1. Initialize the Docker container (`fleetra`).
2. Launch the Gazebo warehouse simulation with two robots.
3. Bring up two RViz telemetry windows.
4. Synchronize initial poses and activate Nav2 lifecycle nodes.
5. Execute the coordinated 4-rack delivery plan.

### 2. Stop the Demo

Press **Ctrl + C** in the terminal running `run_demo.sh`, or execute from another terminal:

```bash
./stop_demo.sh
```

---

## Rebuilding the Docker Image

The Docker image builds ROS 2 Foxy Desktop with a source build of Navigation 2 pinned to the exact commit supported by the warehouse controllers:

```bash
cd CustomRobots/amazon_robot
docker build -f ../../docker/Dockerfile -t fleetra:latest -t jderobot/robotics-backend-ros2:amazon-warehouse-multi-robot .
```

---

## License

This project is licensed under the [MIT License](LICENSE).
