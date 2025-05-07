# Assignment 3: Rocky Times Challenge - Search, Map, & Analyze

This ROS2 package implements an autonomous drone system for detecting, mapping, and analyzing cylindrical geological features using an RGBD camera and PX4 SITL simulation.

---

## 🚀 Challenge Overview

Students develop a controller for a PX4-powered drone to:

- Search and locate cylindrical rock formations.
- Estimate their height and diameter.
- Determine their positions in the world frame.
- Land autonomously on top of the taller cylinder.
- Optimize the mission in terms of time and energy usage.

---

## ✅ Recent Updates by `ryan19f`

| File | Description |
|------|-------------|
| `transform_utils.py` | 🔧 New utility module for frame transforms. |
| `pose_visualizer.py` | 📈 Updated for better live visualization of poses. |
| `geometry_tracker.py` | 📏 Enhanced cylinder geometry estimation logic. |
| `aruco_tracker.py` | 🎯 Improved ArUco-based localization. |
| `cylinder_landing_node.py` | 🛬 Refined autonomous landing on target. |
| `cylinder_landing.launch.py` | 🚀 Simplified launch config with path options. |

---

## 🎯 Mission Objectives

- ✅ Autonomous takeoff and exploration
- ✅ Cylinder detection & dimension analysis
- ✅ World-frame localization
- ✅ Safe, precise landing on tallest cylinder
- ✅ Energy-aware path planning

---

## 🛠 Prerequisites

- ROS2 Humble
- PX4 SITL Simulator (`main` branch `9ac03f03eb`)
- RTAB-Map ROS2
- OpenCV
- Python 3.8+

---

## 🔧 Setup Instructions

### Clone & Symlink

```bash
cd ~/ros2_ws/src
ln -s ~/RAS-SES-598-Space-Robotics-and-AI/assignments/terrain_mapping_drone_control .
```

### PX4 Model Deployment

```bash
cd ~/ros2_ws/src/terrain_mapping_drone_control
chmod +x scripts/deploy_px4_model.sh
./scripts/deploy_px4_model.sh -p /path/to/PX4-Autopilot
```

### Build & Launch

```bash
cd ~/ros2_ws
colcon build --packages-select terrain_mapping_drone_control --symlink-install
source install/setup.bash
ros2 launch terrain_mapping_drone_control cylinder_landing.launch.py
```

---

## 🏆 Evaluation Criteria

- ⏱ Mission time
- 🔋 Energy efficiency
- 📐 Accuracy of measurements
- 🎯 Landing precision
- 🔄 Robustness across 15 trials (10 known + 5 unknown)

---

## ⭐ Extra Credit: 3D Reconstruction (50 pts)

Use RTAB-Map or a SLAM library to generate a mesh of the environment. Upload the mesh using Git LFS if necessary.

---

## 📜 License

This project is licensed under the **Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License**.  
Details: [CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/)
