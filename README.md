# 🏭 warehouse_robots

A ROS 2 package for simulating a 4-wheel warehouse robot in Gazebo (Ignition).  
This package includes the robot description (URDF + meshes), launch files, and a sample warehouse world.

---

## 📦 Package Overview
```
warehouse_robots/
├── launch/               # Gazebo & RViz launch files
├── urdf/                 # URDF / XACRO robot description
├── meshes/               # STL / DAE 3D models
├── worlds/               # Sample warehouse world (.sdf)
├── model.config          # Gazebo model descriptor
├── package.xml
└── setup.py
```

---

## 🚀 Quick Start

### 1️⃣ Clone & Build
```bash
# Create workspace
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src

# Clone this repository
git clone https://github.com/your_username/warehouse_robots.git

# Install dependencies
cd ~/ros_ws
rosdep install --from-paths src -y --ignore-src

# Build
colcon build --symlink-install
source install/setup.bash
```

### 2️⃣ Launch Gazebo Simulation
```bash
ros2 launch warehouse_robots gazebo.launch.py
```

Gazebo will open with:
- the **warehouse world** environment, and  
- the **warehouse robot** spawned at the starting position.

### 3️⃣ Optional – Visualize in RViz
```bash
ros2 launch warehouse_robots display.launch.py
```

---

## ⚙️ Dependencies
- ROS 2 (Humble / Iron / Jazzy)  
- `ros_gz_sim`  
- `robot_state_publisher`  
- `xacro`  
- `rviz2`

Install manually if missing:
```bash
sudo apt install ros-${ROS_DISTRO}-ros-gz ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-robot-state-publisher
```

---

## 🧩 Main Features
- ✅ URDF/Xacro description of a 4-wheel robot  
- ✅ Realistic mesh models (base, wheels)  
- ✅ Gazebo world with warehouse obstacles  
- ✅ Launch integration with `ros_gz_sim`  
- ✅ Easily extendable to include sensors (LiDAR, camera)

---

## 🗺️ Folder Highlights
| Folder | Purpose |
|---------|----------|
| `urdf/` | Robot structure, joints, and links |
| `meshes/` | 3D geometry used in visuals |
| `worlds/` | Pre-built warehouse environment |
| `launch/` | Launch files for Gazebo and RViz |
| `scripts/` *(optional)* | Custom controllers or spawn scripts |

---

## 🧪 Development Notes
- Units: **meters (m)** for all meshes  
- Coordinate: **Z-up**, **X-forward**, **Y-left**  
- To adjust robot size, change `scale` in URDF or re-export STL from SolidWorks (in meters).  
- If Gazebo cannot find meshes, ensure `IGN_GAZEBO_RESOURCE_PATH` includes this package’s share directory.

---

## 🧑‍💻 Maintainer
**Andy Tsai**  
M.S. Robotics & Autonomous Systems @ ASU  
📧 andystsai1040@gmail.com  
🌐 [LinkedIn / GitHub Profile link](https://github.com/your_username)

---

## 🪪 License
MIT License  
See [`LICENSE`](LICENSE) for details.
