# Assignment 3: Rocky Times Challenge – Search, Map & Analyze

This project is a ROS2-based system designed to control an autonomous drone for detecting, localizing, and analyzing cylindrical rock structures using an RGBD camera, all within a PX4 SITL simulation environment.

---

## 🚀 Project Summary

The task was to develop a drone controller that can:

- Search for and identify cylindrical formations in a simulated terrain.
- Estimate each cylinder’s height and diameter.
- Accurately determine their global positions.
- Safely land on top of the tallest cylinder.
- Optimize the mission for efficiency in both time and energy.

---

## ✅ My Recent Contributions

| File | Update Summary |
|------|----------------|
| `transform_utils.py` | 📌 Created a utility for easier frame transformation. |
| `pose_visualizer.py` | 🔍 Enhanced for real-time visualization of drone and object poses. |
| `geometry_tracker.py` | 📐 Improved logic for analyzing cylinder shapes and dimensions. |
| `aruco_tracker.py` | 🧭 Refined ArUco-based localization for more reliable marker tracking. |
| `cylinder_landing_node.py` | 🛬 Updated for smoother autonomous landing execution. |
| `cylinder_landing.launch.py` | ⚙️ Simplified and clarified the launch configuration process. |

---

## 🎯 Main Goals

- ✅ Take off and explore without manual input  
- ✅ Detect and analyze cylindrical geometry  
- ✅ Transform detections into the global coordinate frame  
- ✅ Land precisely on the tallest structure  
- ✅ Conserve energy and minimize mission time

---

## 🛠 Requirements

- ROS2 Humble  
- PX4 SITL (tested with commit `9ac03f03eb`)  
- RTAB-Map for mapping  
- OpenCV  
- Python 3.8 or newer

---

## 🔧 Setup Guide

### Clone & Link the Package

```bash
cd ~/ros2_ws/src
ln -s ~/RAS-SES-598-Space-Robotics-and-AI/assignments/terrain_mapping_drone_control .
```

### PX4 Deployment

```bash
cd ~/ros2_ws/src/terrain_mapping_drone_control
chmod +x scripts/deploy_px4_model.sh
./scripts/deploy_px4_model.sh -p /path/to/PX4-Autopilot
```

### Build and Launch

```bash
cd ~/ros2_ws
colcon build --packages-select terrain_mapping_drone_control --symlink-install
source install/setup.bash
ros2 launch terrain_mapping_drone_control cylinder_landing.launch.py
```

---

## 🏁 Evaluation Breakdown

- ⏱ Mission duration  
- 🔋 Energy usage  
- 📏 Measurement accuracy  
- 🎯 Landing precision  
- 🔁 Reliability over 15 total simulations (10 known + 5 unseen)

---

## 🌐 Bonus Opportunity (50 pts)

For extra credit, I plan to use RTAB-Map or an alternative SLAM tool to generate a 3D mesh of the environment. If the file is large, I’ll upload it via Git LFS.

---

## 📜 Licensing

This project follows the [Creative Commons BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/) license.

---

## 🧪 Debugging Workflow

Here’s how I run and test the nodes during development:

```bash
# 1. Start DDS Agent
MicroXRCEAgent udp4 -p 8888

# 2. Launch GCS (Ground Control Station)
./QGroundControl.AppImage

# 3. Start PX4 simulation
ros2 launch terrain_mapping_drone_control cylinder_landing.launch.py

# 4. Run geometry detection
python3 geometry_tracker.py
ros2 run image_view image_view image:=/geometry/debug_image

# 5. Run ArUco marker tracking
python3 aruco_tracker.py
ros2 run image_view image_view image:=/aruco/debug_image

# 6. Initiate cylinder landing
python3 cylinder_landing_node.py
```

---

## 🔄 State Machine Logic

The mission is managed by a finite state machine that transitions through these stages:

### 1. TAKEOFF  
- Switches to offboard control  
- Arms and ascends to a 5-meter altitude  

### 2. SEARCH  
- Rotates while increasing altitude to locate cylinders  
- Uses yaw-angle mapping to cluster and track objects  
- Ends once only one cylinder remains in view after a full rotation  

### 3. CLIMB  
- Faces the target cylinder and climbs, adding yaw noise for better visibility  
- Measures height at the top of the cylinder  

### 4. APPROACH  
- Positions drone 1.5 meters above the selected cylinder  

### 5. LAND  
- Uses the bottom-facing camera and ArUco tags to guide a precision landing  
- Converts detected marker positions to world coordinates for accurate descent

---

## ⚙️ Technical Challenges & Learning Points

- Setting up PX4, SITL, and QGroundControl was initially tricky but very informative.  
- Working with yaw angles was especially delicate due to the [-π, π] wrapping issue.  
- Adjusting to PX4’s North-East-Down (NED) frame required a shift in thinking.  
- Early ArUco detection was unstable, but adding a `cv2.bilateralFilter` helped with robustness.  
- Tuning mission parameters is ongoing and needs more time for refinement.
