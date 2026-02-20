# Donar Mobile Robot - ROS2 Simulation Package

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Ignition-orange)](https://gazebosim.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A differential drive mobile robot simulation package for ROS2 Humble with stable ball caster design for Gazebo Ignition.

---
<img width="1847" height="955" alt="image" src="https://github.com/user-attachments/assets/5337a20a-5c3d-4ca3-8003-f625923301f5" />
<img width="1834" height="953" alt="image" src="https://github.com/user-attachments/assets/7cbde9a1-4281-41a8-af2a-9176bbb423bb" />


## Project Highlights

- **Stable Differential Drive:** 2 powered wheels with differential drive controller
- **No-Hop Ball Casters:** Redesigned from complex swivel casters to simple ball casters, eliminating reverse motion hopping
- **Dual Format Support:** Both SDF (for Gazebo) and URDF (for RViz/Nav2) formats
- **Production Ready:** Tested and verified in Gazebo Ignition and RViz
- **Extensible Design:** Ready for sensor integration (Lidar, Camera, IMU)

---

## Demo

### Gazebo Simulation
```bash
ros2 launch donar_description gazebo_sdf.launch.py
```

### RViz Visualization
```bash
ros2 run rviz2 rviz2 -d src/donar_description/rviz/urdf_config.rviz
```

### Keyboard Control
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/donar_robot/cmd_vel
```

---

## Quick Start

### Prerequisites

- **OS:** Ubuntu 22.04
- **ROS2:** Humble
- **Gazebo:** Ignition Garden or Fortress
- **Tools:** Colcon, Git

### Installation

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone repository
git clone https://github.com/yourusername/donar_description.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select donar_description

# Source
source install/setup.bash
```

### Usage

**Launch Gazebo Simulation:**
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ros2_ws/install/donar_description/share
ros2 launch donar_description gazebo_sdf.launch.py
```

**Control the Robot:**
```bash
# Install keyboard teleop
sudo apt install ros-humble-teleop-twist-keyboard

# Run teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/donar_robot/cmd_vel
```

---

##  Robot Specifications

| **Parameter** | **Value** |
|---------------|-----------|
| **Type** | Differential Drive |
| **Total Mass** | ~40 kg |
| **Wheel Radius** | 0.14 m |
| **Wheel Separation** | 0.52 m |
| **Max Linear Velocity** | 3 m/s |
| **Max Angular Velocity** | 3.0 rad/s |
| **Caster Type** | Ball (40mm radius) |
| **Control Interface** | geometry_msgs/Twist |

---

## Architecture

### Package Structure
```
donar_description/
├── config/              # Controller configurations
├── launch/              # Launch files (Gazebo, RViz)
├── meshes/              # CAD-exported STL meshes
├── models/              # SDF models for Gazebo
│   └── donar_robot/
│       ├── model.config
│       └── model.sdf    # Primary robot definition
├── rviz/                # RViz configurations
├── urdf/                # URDF/xacro files
└── worlds/              # Gazebo world files
```

### Key Components

**Main Files:**
- `models/donar_robot/model.sdf` - Primary robot definition for Gazebo
- `urdf/donar_robot.xacro` - URDF description for RViz/Nav2
- `launch/gazebo_sdf.launch.py` - Main simulation launcher

**Robot Links:**
- `base_link` - Main robot chassis (35 kg)
- `left_wheel_1`, `right_wheel_1` - Driven wheels (2 kg each)
- `left_caster_wheel_1`, `right_caster_wheel_1` - Ball casters (0.4 kg each)

**Topics:**
- `/donar_robot/cmd_vel` - Velocity commands (input)
- `/donar_robot/odometry` - Odometry feedback (output)
- `/joint_states` - Joint positions/velocities (output)

---

---

##  Development Roadmap

###  Completed
- [x] Base robot model with differential drive
- [x] Simplified ball caster design
- [x] Gazebo Ignition integration
- [x] RViz visualization support
- [x] Odometry and joint state publishing
- [x] Teleop control interface

###  In Progress
- [ ] RPLidar A2 sensor integration
- [ ] Intel RealSense camera
- [ ] IMU sensor (for sensor fusion)

###  Planned Features
- [ ] Nav2 navigation stack integration
- [ ] SLAM capability (slam_toolbox)
- [ ] Autonomous waypoint navigation
- [ ] Gazebo world scenarios
- [ ] Hardware deployment guide

---

## Documentation

### Additional Resources

- **ROS2 Tutorials:** [docs.ros.org](https://docs.ros.org/en/humble/)
- **Gazebo Documentation:** [gazebosim.org](https://gazebosim.org/docs)
- **Project Wiki:** Coming soon

### API Reference

**Published Topics:**
```
/donar_robot/odometry [nav_msgs/Odometry]
/joint_states [sensor_msgs/JointState]
/clock [rosgraph_msgs/Clock]
/tf [tf2_msgs/TFMessage]
```

**Subscribed Topics:**
```
/donar_robot/cmd_vel [geometry_msgs/Twist]
```

**Parameters:**
```yaml
wheel_separation: 0.52
wheel_radius: 0.14
max_linear_velocity: 3
max_angular_velocity: 3.0
```

---

##  Contributing

Contributions are welcome! Please feel free to submit issues and pull requests.

### Development Setup

```bash
# Fork the repository
# Clone your fork
git clone https://github.com/Fonyuy45/donar_description.git

# Create feature branch
git checkout -b feature/your-feature

# Make changes and commit
git commit -m "feat: add your feature"

# Push and create PR
git push origin feature/your-feature
```

---

##  License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 👤 Author

**Dieudonne YUFONYUY**

- Email: dieudonne.yufonyuy@gmail.com
- LinkedIn: [[https://www.linkedin.com/in/dieudonne-yufonyuy](https://www.linkedin.com/in/dieudonne-yufonyuy)

---

## Acknowledgments

- **ROS2 Community** - For excellent documentation and support
- **Gazebo Team** - For powerful simulation tools
- **fusion2urdf** - For CAD to URDF conversion
- Open source robotics community

---

## Project Stats

![GitHub last commit](https://img.shields.io/github/last-commit/yourusername/donar_description)
![GitHub issues](https://img.shields.io/github/issues/yourusername/donar_description)
![GitHub stars](https://img.shields.io/github/stars/yourusername/donar_description?style=social)

---

## Related Projects

- [Navigation2](https://github.com/ros-planning/navigation2) - ROS2 Navigation Stack
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) - SLAM for ROS2
- [gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs) - Gazebo ROS integration

---

**⭐ If you find this project useful, please consider giving it a star!**

*Last Updated: December 2025*
