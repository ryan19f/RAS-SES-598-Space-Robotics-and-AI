What I'm Doing:-
In this assignment, I’ll work with the provided ROS2 code using Turtlesim to refactor and tune a navigator for a precise lawnmower survey (also called a boustrophedon pattern). The code I’m starting with produces a non-uniform pattern (like the one shown above). My task is to dig into how proper lawnmower surveys are designed and modify the navigator to create a uniform, systematic survey pattern.

Why This Matters
Boustrophedon patterns (literally meaning “ox-turning,” like plowing a field) are super important for coverage tasks in many areas:

Space Exploration
Rovers use these patterns to fully survey areas when mapping terrain or looking for geological samples. In energy-constrained environments, optimized versions of these paths balance coverage with efficiency.
Earth Observation
Drones and other aerial vehicles rely on these patterns for:
Precision farming
Search and rescue missions
Environmental and geological surveys
Ocean Exploration
Autonomous underwater vehicles (AUVs) use these patterns for tasks like:
Mapping the seafloor
Searching for wreckage
Monitoring marine ecosystems
The performance of these surveys depends on how accurately the robot follows its path with minimal cross-track error. This assignment simulates those real-world challenges using Turtlesim in a controlled 2D environment.

My Goal
I’ll tune a PD controller to get the Turtlesim robot to execute the most precise boustrophedon pattern possible. The focus is to minimize cross-track error while ensuring smooth movement.

What I Need to Know First
System Setup

I’ll be using one of the following configurations (depending on my machine):

Ubuntu 22.04 + ROS2 Humble
Ubuntu 23.04 + ROS2 Iron
Ubuntu 23.10 + ROS2 Iron
Ubuntu 24.04 + ROS2 Jazzy
Software Requirements

I’ll make sure I have these installed:

sudo apt install ros-$ROS_DISTRO-turtlesim
sudo apt install ros-$ROS_DISTRO-rqt*
Python Tools

To help with analysis and visualization, I’ll also install these Python libraries:

pip3 install numpy matplotlib
What I’m Learning
How PD Control Works
I’ll dive into the effects of proportional and derivative parameters on first-order systems and learn how they influence tracking performance.
Controller Tuning
I’ll practice adjusting the PD controller to balance smooth motion and precise tracking.
Performance Analysis
Using ROS2 tools and Python visualizations, I’ll analyze how well the robot follows the desired path.
ROS2 Debugging
I’ll get hands-on experience visualizing and debugging in ROS2.

## The Challenge

### 1. Controller Tuning (60 points)
Use rqt_reconfigure to tune the following PD controller parameters in real-time:
'''python

Controller parameters that I tuned
self.Kp_linear = 5.0   # Proportional gain for linear velocity
self.Kd_linear = 0.5   # Derivative gain for linear velocity
self.Kp_angular = 9.5  # Proportional gain for angular velocity
self.Kd_angular = 0.01  # Derivative gain for angular velocity
'''

This was done using a trial-and-error method, checking various parameters to see if it performs according to the expected outcome. By doing this, we tested out the various limitations that the total has in certain areas by navigating edge to edge.



