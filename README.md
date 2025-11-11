# 🏭 warehouse_robots

A ROS 2 package for simulating a 4-wheel warehouse robot in Gazebo (Ignition).  
This package includes the robot description (URDF + meshes), launch files, and a sample warehouse world.

---

## 📦 Package Overview
```
warehouse_robots/
├── launch/               # Gazebo & RViz launch files
├── warehouse_robots/     # Python ROS 2 nodes (executable scripts)
│   ├── __init__.py
│   ├── ...
│   └── ...
├── resource/
│   └── warehouse_robots
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
- ROS 2 (Humble)  
- `ros_gz_sim`  
- `robot_state_publisher`  
- `xacro`  
- `rviz2`

Install manually if missing:
```bash
sudo apt install ros-humble-ros-gz ros-humble-xacro ros-humble-robot-state-publisher
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
| `warehouse_robots/` | Custom controllers and robot's functions |

---

## 🧑‍💻 Maintainer
**Andy Tsai**  
M.S. Robotics & Autonomous Systems @ ASU  
📧 andystsai1040@gmail.com  
🌐 [LinkedIn link](https://www.linkedin.com/in/chih-hao-tsai/)
🌐 [Github Profile](https://github.com/andytsai104)

---

## 🧪 TODO:
1. 重新設計倉庫大小
2. 重新設計機器人大小（約長寬1m）
3. 設計機器人基本功能（PID controller: 直走，轉彎，停止...）
4. 寫.rviz file (預設robot model內的description=robot_description, etc.)

### - Alan:
1. Robot's PID
2. Task distributer
3. 指令switcher node

### - Andy:
1. 中央黑板節點(分散式數據廣播)
2. RL training 跟 RL-based decision maker
3. Reward function 定義

###　- Quinn:
1. A*
2. Consensus decision maker
3. 數據記錄器 (Metric logger)


---
