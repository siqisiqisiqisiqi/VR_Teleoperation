# 🤖 VR-Controlled Humanoid Robot with Inverse Kinematics

This project integrates **real-time VR teleoperation** (via Meta Quest) and a **closed-loop inverse kinematics (CLIK) solver** using **Pinocchio** to control a humanoid robot's upper body in simulation (Isaac Sim) through ROS 2.
A test Isaac Sim environment can be found [here](https://drive.google.com/file/d/1Bsv98zKYRY3rpAmXcEab5VOmEmICtvBd/view?usp=drive_link)

---

## 📁 Repository Structure

```
.
├── pinocchio_ik_solver/   # CLIK-based IK solver with joint-space control
├── TeleVision/            # VR interface using Meta Quest + ROS 2 bridge
├── data_collection/       # ROS-based data collection and convertion for lerobot format dataset
└── .gitignore
```

---

## 🧠 Project Overview

The project enables you to:

- Receive VR hand/head/finger pose from Meta Quest (via WebSocket/ADB)
- Use a Pinocchio-based CLIK controller to compute joint commands
- Publish those to Isaac Sim via ROS 2 for real-time humanoid control

---

## 🔧 Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/your_username/your_repo_name.git
   cd your_repo_name
   ```

2. **Setup ROS 2 workspace**
   ```bash
   cd pinocchio_ik_solver
   colcon build
   source install/setup.bash
   ```

3. **Install Python dependencies following each packages instruction**
   (Ensure Python 3.10+ is used)


---

## 🚀 How to Run
This is just a workflow demo, please check each submodule for detail implementation.
### 1. Launch VR Bridge
Make sure Meta Quest is connected and run:

```bash
cd TeleVision
python3 teleop_hand.py
```

### 2. Start IK Solver
Make sure Isaac Sim and ROS 2 nodes are running. Then:

```bash
python3 clik_vr_ros.py
```
### 3. ROS data collection
Make sure Isaac Sim and ROS 2 nodes are running. Then:

```bash
python3 data_collection.py
```

---


## 📄 Submodules Documentation

Each subdirectory contains its own `README.md` file for details:
- [`pinocchio_ik_solver/README.md`](./pinocchio_ik_solver/README.md)
- [`TeleVision/README.md`](./TeleVision/README.md)
- [`data_collection/README.md`](./data_collection/README.md)
---

## 🤝 Acknowledgements

- [Pinocchio](https://github.com/stack-of-tasks/pinocchio)
- [Meta Quest OpenXR](https://developer.oculus.com/)
- [Isaac Sim](https://developer.nvidia.com/isaac-sim)

---

## 📌 License

MIT License. See `LICENSE` file for details.
