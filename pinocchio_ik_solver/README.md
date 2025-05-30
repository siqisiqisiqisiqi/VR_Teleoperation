
# 🤖 Pinocchio IK Solver with VR/Slider Input & Isaac Sim Integration: IK Solver Section

This project implements a **Closed-Loop Inverse Kinematics (CLIK) solver** using [Pinocchio](https://github.com/stack-of-tasks/pinocchio) to control a 7-DOF humanoid arm (`T170A`) in real-time. It is controlled through ROS2:

- ✅ **VR controller** (Meta Quest3)
- ✅ **Pose slider GUI** (manual override or debugging)
- ✅ **Meshcat** (for visualization)
- ✅ Integration with **Isaac Sim** for physics-based simulation

---

## 📁 Project Structure

```
pinocchio_ik_solver/
├── models/t170a_description/    # URDF, SRDF, and meshes
├── pinocchio_ik_solver/
│   ├── clik_vr_ros.py           # ROS 2 node to perform IK with VR input
│   ├── clik_slider_meshcat.py   # Run IK with GUI sliders + Meshcat
│   ├── slider_gui_pose.py       # Standalone GUI pose slider (Pose publisher)
├── setup.py
├── package.xml
```

---

## 🚀 Installation

### ✅ 1. Set up ROS 2 workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <this_repo> pinocchio_ik_solver
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

> Make sure you have `pinocchio`, `pyquaternion`, `PyQt5`, `scipy`, `rclpy`, and `meshcat` installed in your Python (Python=3.10) environment.

Install Python dependencies:

```bash
pip install pin numpy scipy PyQt5 meshcat
```

---

## 🧠 Run CLIK Solver with Different Input Sources

### ▶️ CLIK + Meshcat + Slider GUI without ROS (Debug Version)

Run the manual GUI and IK loop:

```bash
python clik_slider_meshcat.py
```

- GUI lets you slide pose inputs (x, y, z, roll, pitch, yaw)
- Meshcat displays the arm and target
- Useful for debugging IK behavior in isolation

---

### ▶️ CLIK (ROS Version)


```bash
python slider_gui_pose.py
python clik_vr_ros.py
```
- Subscribes to the robot joint state topic (`/joint_states_isaac`) (from Isaac Sim or real robot)
- Subscribes to a VR pose topic (`/hand_pose_ik`)
- Publishes joint angles to `/joint_command` (to Isaac Sim or real robot)
- Uses Pinocchio for FK, IK, and collision avoidance

---


## 🧪 Isaac Sim Integration Notes

- Isaac Sim must publish `/joint_states_isaac` and subscribe to `/joint_command`
- Isaac Sim FPS should be higher than the Pinocchio controller FPS.
- Adjust stiffness/damping in Isaac Sim as needed

---
## 🎥 Demo

![CLIK Demo](docs/meshcat_pinocchio_IK_solver.gif)

## ✅ Coming Soon

- [ ] Unified `launch.py` for all components
- [ ] Integral control for CLIK (optional I-term)
- [ ] VLA (vision-language-action) demo integration

---

## 📝 License

This project is under the MIT License.
