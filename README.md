# Project Seek and Destroy (the Boredom)

**EEEE 585/685 — Principles of Robotics | Spring 2026 | Track A: Mobile Robot**  
Joyce Keeriath · Garima Singh

An autonomous robot mission system built on ROS 2 Humble with Ignition Gazebo.
The robot starts in a completely unknown environment, explores autonomously using
frontier-based exploration, detects color-coded human target figures using HSV
computer vision, confirms their map-frame location, returns home, and saves an
annotated map with all detected targets marked on it.

> Demo video — https://drive.google.com/file/d/1D-IGr35N3BSuqlBnd13syy1ILCa6T-Q4/view?usp=sharing
> Full Project Documentation - https://docs.google.com/document/d/1Zf_-dKFGVWF04KVgigR9BlpbmcfZSNFWzu57KjYDwf4/edit?usp=drive_link 

---

## Quick Start

```bash
# 1. Every new session — run this first
cd ~/seek_destroy_ws/src && ./setup_session.sh

# 2. Every new terminal — source the workspace
cd ~/seek_destroy_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib/ign_ros2_control/system:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH

# 3. Build
colcon build --symlink-install
source install/setup.bash

# 4. Kill any leftover processes before launching
pkill -9 -f "ros|gz|ign|rviz"; sleep 5

# 5. Launch everything
ros2 launch seek_and_destroy_sim mission.launch.py
```

Wait ~25 seconds for all nodes to initialize. The Mission Control GUI and 
xterm state machine window will appear automatically.

---

## How to Run a Mission

**Using the GUI (recommended):**
1. Wait for the Mission Control GUI window to appear
2. Click Red Cylinder, Blue Box, or Green Cylinder to start
3. The robot runs the full mission autonomously — no further input needed
4. Monitor state, confidence, and logs in the dashboard

**Using the xterm terminal (fallback):**
1. Click inside the small xterm window
2. Type the target name: `redcylinder`, `bluebox`, or `greencylinder`
3. Press Enter

**After the mission:**
- Click **Search Again** or type `s` to start a new mission
- Click **Quit Mission** or type `q` to shut down cleanly

---

## What the Robot Does

| State | What Happens |
|-------|-------------|
| `SWEEPING` | 360° spin to scan immediate surroundings before exploring |
| `EXPLORING` | Frontier-based autonomous mapping via `explore_lite` |
| `INVESTIGATING` | Drives toward low-confidence detections to confirm or reject |
| `ROAMING` | Visits obstacle-proximity waypoints when frontiers exhausted |
| `HOMING` | Returns to (0, 0) after target confirmed |
| `SAVING_MAP` | Saves map PNG with colored target markers overlaid |

Saved maps go to `~/seek_destroy_ws/src/saved_maps/`.

---

## Package Structure
```text
seek_destroy_ws/src/
├── seek_and_destroy_brain/        # All custom ROS 2 nodes
│   ├── color_detector.py          # HSV detection + map-frame localization
│   ├── state_machine.py           # Mission brain + state graph
│   ├── navigator.py               # Nav2 action client
│   ├── mission_gui.py             # Tkinter Mission Control GUI
│   └── config/targets.yaml        # HSV ranges per target
├── seek_and_destroy_sim/          # Simulation package
│   ├── worlds/lab.sdf             # Gazebo world with human figures + patrol person
│   ├── launch/mission.launch.py   # Single launch file for everything
│   └── config/nav2_params.yaml    # Nav2 configuration
├── saved_maps/                    # Annotated map output
└── docs/
└── seek_and_destroy_report.pdf
```
## Original Contribution

All four nodes in `seek_and_destroy_brain/` are entirely student-authored:

- **`state_machine.py`** — full mission state graph with whitelisted transitions,
  confidence integrator with exponential decay, investigation drive, obstacle-based
  roaming via OpenCV morphological processing, session-persistent target memory,
  initial 360° sweep, and OpenCV map annotation pipeline
- **`color_detector.py`** — HSV detection pipeline with dual localization methods:
  RGB-D back-projection (Method A) and LiDAR ray-casting (Method B)
- **`navigator.py`** — Nav2 action client handling six command types with
  per-goal-type status reporting
- **`mission_gui.py`** — tkinter Mission Control GUI running as a ROS 2 node
  with live confidence bar, target selection, mission log, and dynamic obstacle spawner

The simulation world (`lab.sdf`) including human-shaped target figures and the
patrolling dynamic obstacle, and all Nav2 YAML configuration fixes were also
student-authored.

---

## Useful Commands

```bash
# Check what nodes are running
ros2 node list

# Check all ROS/Gazebo processes
ps aux | grep -iE "ros|gz|ign|rviz"

# Full process wipe
pkill -9 -f "ros|gz|ign|rviz"

# If something is still stuck
ros2 daemon stop && ros2 daemon start

# Convert saved map PGM to PNG manually
convert map1.pgm map1.png

# Check available rosbot packages
ros2 pkg list | grep -i rosbot
```

---

## Safety

- Fully simulation-based — no physical hardware
- Emergency Stop in the GUI pauses exploration and sends the robot home immediately
- Closing the GUI or xterm triggers full system shutdown
- Always use **Quit Mission** or `Ctrl+C` in the launch terminal for clean shutdown
- Never kill individual nodes manually during an active mission

---

## AI Disclosure

AI tools (Claude by Anthropic, Gemini) were used to assist with documentation,
debugging ROS 2 configuration issues, and code structure suggestions.
All system architecture, node logic, state machine design,
detection pipeline, test cases, and ideas were conceived and implemented by the
students. No AI-generated code was used without full review and understanding.

---

## External Code Disclosure

| Component | Source |
|-----------|--------|
| `explore_lite` | [robo-friends/m-explore-ros2](https://github.com/robo-friends/m-explore-ros2) |
| `slam_toolbox` | Standard ROS 2 Humble package |
| `nav2_bringup` | Standard ROS 2 Navigation2 stack |
| `ros_gz_sim` | Ignition Gazebo ROS 2 bridge |
| `rosbot_gazebo` | Husarion ROSbot simulation package |

All code in `seek_and_destroy_brain/` is original student work.
