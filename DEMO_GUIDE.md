# Fleetra – Multi-Robot Autonomous Warehouse Orchestration Demo Guide

Fleetra orchestrates autonomous mobile robots (AMRs) in warehouse environments using ROS 2 Foxy, Nav2, Gazebo 11, and CycloneDDS.
Simulation baseline: 2 robots in the AWS small warehouse world (`aws_amazon_robot_multiple.py`), executing sequential coordinated tasks (`src/exercise/multi-robot.py` → `create_test_plan()`).

## Start & Stop

```bash
~/Fleetra/run_demo.sh      # starts everything; Ctrl+C stops cleanly
~/Fleetra/stop_demo.sh     # stop from another terminal
```

`run_demo.sh` performs the following automated steps:
1. Configures `xhost +local:docker`, spins up the `fleetra` container (Intel/Mesa: `--device /dev/dri`; switches to `--gpus all` if NVIDIA container runtime is present), `--net=host`, mounts X11 socket, display, scripts, and params.
2. Launches Gazebo world, robots, and Nav2 stacks:
   `. ./install/setup.sh && ros2 launch amazon_robot_bringup aws_amazon_robot_multiple.py robot1_params_file:=/demo_params/nav2_multirobot_params_1_with_control.yaml robot2_params_file:=/demo_params/nav2_multirobot_params_2_with_control.yaml map:=/demo_params/map_3beds/map.yaml`
3. Polls until both robots are spawned and the Nav2 lifecycle managers exist.
4. Programmatically configures RViz / Nav2 states:
   - `scripts/activate_nav2.py --stage localization` (RViz Nav2 localization transition)
   - `scripts/set_initial_poses.py` (reads Gazebo pose, refines lidar scan against map, publishes initial pose)
   - `scripts/activate_nav2.py --stage navigation` (transitions Nav2 lifecycle to active navigation)
5. Executes the coordination sequencer: `python3 src/exercise/multi-robot.py`.
6. Ctrl+C clean teardown: stops running nodes, terminates container, and resets X11 permissions.

Logs: `~/Fleetra/logs/` (`launch_*.log`, `plan.log`).

## Visualization & UI

- **Gazebo**: AWS small warehouse environment with 2 robots (differentiated with visual 7-segment numeral plates "1" and "2" on their lift mechanisms) and yellow pallet racks.
- **2 RViz Windows** (`robot1`, `robot2`): Map, global/local costmaps, lidar ray casts, AMCL particle clouds, and planned paths.
- Approximate timing:
  - Container + Gazebo spawn: ~5-8 s
  - Lidar box filter + Nav2 activation + initial pose alignment: ~15–20 s
  - Autonomous task execution commences: ~25–30 s after `run_demo.sh`
  - First rack lifted: ~15 s after task plan execution starts.

## Architectural Improvements & Fixes

1. **Dockerized Environment & Build**:
   `docker/Dockerfile` reproduces the complete underlay and overlay requirements: `osrf/ros:foxy-desktop` + Navigation2 main @ `98e994f` (Aug 2020) + BehaviorTree.CPP & bond_core lifecycle support + warehouse workspace in `/opt/warehouse_ws`.
   To rebuild locally:
   ```bash
   cd ~/Fleetra/CustomRobots/amazon_robot && docker build -f ~/Fleetra/docker/Dockerfile -t fleetra:latest -t jderobot/robotics-backend-ros2:amazon-warehouse-multi-robot .
   ```
2. **DDS Middleware: CycloneDDS on Localhost Loopback** (`scripts/cyclonedds_localhost.xml`):
   Fast-DDS discovery under high node density (~80 ROS nodes across 2 Nav2 stacks, Gazebo, and 2 RViz instances) can cause dropped discovery packets. CycloneDDS configured for loopback unicast guarantees stable node discovery regardless of host network interfaces.
3. **Automated Gazebo Initial Pose & LiDAR Matching**:
   Eliminates manual RViz "2D Pose Estimate". Reads real-time Gazebo ground-truth pose, aligns 2D LiDAR readings against the grid map, and publishes `/<robot>/initialpose` until AMCL converges.
4. **Dynamic LiDAR Box Filter** (`scripts/rack_scan_filter.py`):
   Remaps `/robotN/scan` to `/robotN/scan_filtered`, filtering out sensor returns within the 2.5 × 1.2 m footprint of a carried pallet rack. Prevents robots from identifying carried cargo as obstacles.
5. **Nav2 Parameter Optimization** (`params/`):
   - Goal tolerances tightened (0.08 m / 0.08 rad) for precise alignment under storage racks.
   - Costmaps configured with `inf_is_valid: True` so filtered lidar rays clear stale obstacles left behind by moved racks.
   - AMCL diffusion parameters (`alpha1..5: 0.02`) tuned to minimize drift in large aisle corridors.
6. **Task Sequencer & Coordination**:
   `Sequencer` class in `multi-robot.py` provides turn-taking, staging points (passage wait, aisle approach vectors), and retry logic for pick, transport, and drop-off jobs.

## Troubleshooting Reference

| Symptom | Cause & Solution |
|---|---|
| `Demo is already running` | Stale lock file or running container. Run `~/Fleetra/stop_demo.sh`. |
| DDS discovery timeouts | Ensure `scripts/cyclonedds_localhost.xml` is mounted and `CYCLONEDDS_URI` is set. |
| Robot stops during pallet transport | Ensure `rack_scan_filter.py` is running and publishing on `/robotN/scan_filtered`. |
| Second job aborted near racks | Verify `inf_is_valid: True` in obstacle layer params. |
