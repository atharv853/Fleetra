# How to Run Fleetra - Multi-Agent Warehouse Robot Demo

Two autonomous mobile robots work cooperatively in a warehouse. Each robot navigates under a yellow storage rack, lifts it, carries it across the facility to an assigned destination spot, and safely lowers it down. The entire orchestration is automated with a single command.

---

## Preparation (do this once, 5 minutes early)

1. Turn on the computer and log in.
2. Connect the charger (simulation and multi-robot navigation are compute-intensive).
3. Close background apps with high CPU/memory usage.
4. Open a **Terminal** (press Super/Windows key, type `Terminal`, press Enter).
5. Do a quick test run (see "Start the demo" below), then stop it to ensure images and displays are cached.

---

## Start the demo

**Step 1:** In your terminal, run:

```bash
~/Fleetra/run_demo.sh
```

**Step 2:** Wait without clicking. You'll see automated progress in the terminal:

```
[fleetra    0s] Creating container fleetra
[fleetra    1s] Launching Gazebo warehouse + 2 robots + 2 RViz
[fleetra    8s] both robots spawned in Gazebo: ok
[fleetra   10s] Nav2 lifecycle managers up: ok
[fleetra   18s] lidar box filters publishing: ok
[fleetra   20s] Setting initial poses
[fleetra   29s] Running src/exercise/multi-robot.py ...
```

**Step 3:** Three windows open automatically:

| Window | What it shows |
|---|---|
| **Gazebo** (3D View) | The warehouse environment, the 2 robots (with laser scans and 7-segment numbered IDs) and yellow racks |
| **RViz – robot1** | Live sensor data, costmaps, global/local paths, and AMCL localization for robot 1 |
| **RViz – robot2** | Live sensor data, costmaps, global/local paths, and AMCL localization for robot 2 |

**Step 4:** About **30 seconds** after launch, the robots begin executing tasks. Bring the **Gazebo** window to the front.

---

## Execution Workflow (about 3 minutes)

1. **robot1** drives to the first rack, positions underneath, and **lifts** it.
2. **robot2** drives down the aisle to another rack and lifts it.
3. Each robot **transports** its rack across the warehouse to designated drop-off locations and **unloads**.
4. Robots return to collect subsequent racks and deliver them according to plan.
5. In total, **4 racks are moved (2 per robot)** safely without collision or deadlocks.

### Talking Points

- "Two autonomous mobile robots share a dynamic warehouse floor using ROS 2 Navigation (Nav2)."
- "Each robot performs real-time localization with 2D LiDAR and plans collision-free global and local paths."
- "The Fleetra task sequencer schedules pickup and delivery orders with aisle standoff and turn-taking logic."
- "All lifting, transport, and drop-off sequences occur without human intervention."

---

## Stop the demo

- Switch to the terminal running the demo and press **Ctrl + C**.
- Wait ~10 seconds for clean container teardown.
- The terminal displays `Stopped.`

If the terminal was accidentally closed, open a new terminal and run:

```bash
~/Fleetra/stop_demo.sh
```

---

## Restarting

First stop the current execution, then re-run:

```bash
~/Fleetra/run_demo.sh
```

---

## Troubleshooting

| Problem | Solution |
|---|---|
| **"Fleetra demo is already running"** | Run `~/Fleetra/stop_demo.sh`, wait 5 seconds, and start again. |
| **TIMEOUT waiting for component** | Re-run `~/Fleetra/run_demo.sh`. |
| **Robot idle for > 1 minute** | Press **Ctrl + C**, wait 10 seconds, and restart. |
| **Gazebo rendering appears slow** | Physics runs at real-time factor ~0.95; robots navigate at normal speed even when rendering on CPU. |
| **RViz panel shows "Navigation: inactive"** | Expected behavior. Nav2 lifecycle states are programmatically managed by Fleetra scripts. |
| **No windows appear** | Ensure you are logged into a local graphical session (X11/Wayland). |

---

## Quick Reference

```
START :  ~/Fleetra/run_demo.sh
WAIT  :  ~30 seconds until autonomous motion begins
SHOW  :  Gazebo 3D simulation window
TIME  :  ~3 minutes for full 4-rack orchestration
STOP  :  Ctrl + C   (or ~/Fleetra/stop_demo.sh)
```
