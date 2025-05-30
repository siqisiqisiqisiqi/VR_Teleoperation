
# 🧠🤖 TeleOp Arm Pin: Meta Quest Teleoperation to ROS2 + Pinocchio

This project integrates **Meta Quest hand tracking** with **ROS 2** to control a robot arm using a Pinocchio-based inverse kinematics controller. It enables immersive VR teleoperation using the **TeleVision** interface with stereo camera input and publishes pose and joint commands to ROS.

---

## 🧩 Features

- Meta Quest streaming via TeleVision
- Stereo camera input (left & right)
- Converts wrist & hand pose into ROS2 `Pose` and `JointState` messages
- Supports IK control (via `/hand_pose_ik`) and normal pose control (`/hand_pose`)
- Gripper joint mapping included
- Head orientation sent as joint states

---

## 📦 Installation

Make sure you have:

- Python ≥ 3.8
- ROS 2 (e.g., Humble)
- Dependencies from `requirements.txt` (TeleVision, pytransform3d, cv_bridge, numpy, etc.)

```bash
sudo apt install ros-humble-cv-bridge
pip install -r requirements.txt
```

---

## 🚀 Running the Teleop Code

1. Make sure your Meta Quest is connected via ADB (USB or Wi-Fi)
2. Start ROS 2:
```bash
source install/setup.bash
ros2 run your_package_name teleop_arm_pin
```

3. Ensure `/tf_right_wrist`, `/rgb_left`, `/rgb_right` topics are active
4. IK Pose is published to: `/hand_pose_ik`

---

## 📁 File Overview

- `teleop_arm_pin.py`: Main script to process VR data and publish to ROS
- `inspire_hand.yml`: Retargeting configuration
- `assets/`: URDF files for hand retargeting

---

## 📷 Teleoperation Pipeline

Meta Quest ➝ TeleVision ➝ Retargeting ➝ ROS2 `/hand_pose_ik` ➝ Isaac Sim or Robot

---

For ADB + Meta Quest setup: see `metaquest_connection.md`

