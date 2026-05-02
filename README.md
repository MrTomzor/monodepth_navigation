# Monocular Vision-Based UAV Navigation in Large-Scale Unstructured Environments

A monocular camera-based navigation pipeline for UAVs in unstructured environments. The system converts relative depth maps from [MiDaS](https://github.com/isl-org/MiDaS) models into metric estimates using sparse VIO landmarks from OpenVINS, and uses the resulting point cloud for either reactive navigation or OctoMap-based path planning.

**Bachelor's Thesis** — Czech Technical University in Prague, Faculty of Electrical Engineering, Department of Cybernetics

---
<table>
  <tr>
    <td align="center" width="25%"><b>RGB Image</b></td>
    <td align="center" width="25%"><b>MiDaS Depth Map</b></td>
    <td align="center" width="25%"><b>OpenVINS Landmarks</b></td>
    <td align="center" width="25%"><b>Point Cloud</b></td>
  </tr>
  <tr>
    <td><img src="fig/rgb_input.jpg"        width="100%" style="aspect-ratio:4/3;object-fit:cover"/></td>
    <td><img src="fig/depth_estimation.jpg" width="100%" style="aspect-ratio:4/3;object-fit:cover"/></td>
    <td><img src="fig/vio_input.jpg"        width="100%" style="aspect-ratio:4/3;object-fit:cover"/></td>
    <td><img src="fig/pointcloud.jpg"       width="100%" style="aspect-ratio:4/3;object-fit:cover"/></td>
  </tr>
</table>

---

## Repository Structure

```
├── ros1/            # ROS 1 (Noetic)
└── ros2/            # ROS 2 (Jazzy)
```

> The `ros1` branch contains the original ROS 1 Noetic implementation used during early development.  
> The `ros2` branch is the current, actively maintained version.

---

## Requirements

- ROS 2 Jazzy
- Python 3.12+
- [FlightForge simulator](https://github.com/ctu-mrs/flight_forge)
- [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system)
- [OpenVINS (MRS fork)](https://github.com/ctu-mrs/open_vins)

---

## Installation

**1. Clone into your ROS 2 workspace:**
```bash
cd ~/ros2_ws/src
git clone <repo-url>
cd monodepth_navigation
```

**2. Build the workspace:**
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

**3. Set up the Python environment:**
```bash
./create_python_env.sh
```

This script installs `python3-venv`, creates the virtual environment with `--system-site-packages`, and installs all dependencies from `requirements.txt`.

To update dependencies in an existing environment:
```bash
./install_requirements.sh
```

---

## Running in Simulation

The pipeline is evaluated using the [FlightForge](https://github.com/ctu-mrs/flight_forge) simulator. For more details about the simulator, see the [FlightForge documentation](https://ctu-mrs.github.io/docs/simulations/FlightForge/configuration).

**1. Launch a simulation world:**
```bash
cd tmux/unreal/forest 
./start.sh
```

Available worlds:
```
tmux/unreal/forest/
tmux/unreal/infinity_forest/
tmux/unreal/electrical_towers/
```

**2. Configure the simulation:**

- **`session.yml`** — tmux session layout, launched nodes, UAV name, navigation goal
- **`config/`** — sensor parameters, OctoMap settings, etc.

---


## Launch Files

The pipeline has two main launch files in the `launch/` directory: `monodepth.launch.py` for the depth estimation node and `navigation.launch.py` for the navigation node.

In `monodepth.launch.py`, you can configure the input and output topics.

In `navigation.launch.py`, the navigation mode is controlled by the `is_reactive` parameter — set it to `true` for reactive navigation or `false` for OctoMap-based planning. The input point cloud topic should match the output of the depth estimation node. 

---

## Navigation Modes

### Reactive Navigation 
Simple forward-flight with obstacle avoidance based on sector-based point cloud analysis. The UAV flies forward and rotates away from obstacles when detected within configurable distance thresholds. Constrained to a horizontal plane.

### OctoMap-Based Planning 
The UAV builds a 3D occupancy map incrementally and uses it to plan collision-free paths toward a goal position. Supports full 3D navigation including vertical adjustments.

