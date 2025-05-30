
# TeleOpeartion with ROS integration
## Introduction
This code contains implementation for teleoperation, more detail manipulation can be found [here](https://medium.com/@lazerwalker/how-to-easily-test-your-webvr-and-webxr-projects-locally-on-your-oculus-quest-eec26a03b7ee).

## Installation
Try to install the whole environment in Python 3.10.
```bash
    conda create -n tv python=3.10
    conda activate tv
    pip install -r requirements.txt
```


## 🎮 Meta Quest ADB Setup Guide

This guide explains how to connect your Meta Quest to your computer using **USB ADB** or **Wi-Fi ADB**, and how to set up **reverse port tunneling** for applications like TeleVision.

---

### ✅ Daily Startup Procedure

#### 1. **Power on your Quest**

- Make sure it’s turned on
- Ensure it’s connected to **Wi-Fi with DHCP**

---

#### 2. **USB ADB Connection**  
(*or skip to step 3 for Wi-Fi*)

```bash
adb kill-server
adb start-server
adb devices
```

- On first run, you may see: `unauthorized`
- **Put on the Quest headset**, accept the “Allow USB debugging?” dialog
- Then rerun:

```bash
adb devices  # should now list "device"
```

---

#### 3. **(Optional) Switch to Wireless ADB**

If you'd like to unplug the USB cable:

```bash
# still over USB:
adb tcpip 5555

# find the Quest's IP address:
adb shell ip route
adb shell ifconfig wlan0

# unplug the USB cable, then:
adb connect <quest-ip>:5555     # e.g. 192.168.2.179:5555
adb devices                     # should now list "<ip>:5555 device"
```

---

#### 4. **Reverse-Tunnel TeleVision Port**

If you're using TeleVision (or another service that listens on port `8012`):

```bash
adb reverse tcp:8012 tcp:8012
```

This tunnels the port from your computer to the Quest so apps like TeleVision can communicate.

---

### 🔁 Notes

- Use `adb usb` to return to USB mode.
- Use `adb disconnect` to stop Wi-Fi ADB connections.
- Always verify `adb devices` to ensure the connection is active.

---
## 🚀 Running the Teleop Code

1. Make sure your Meta Quest is connected via ADB (USB or Wi-Fi)
2. Go to the corresponding folder and run:
```bash
cd teleop
python teleop_arm_pin.py
```

3. Ensure `/tf_right_wrist`, `/rgb_left`, `/rgb_right` topics are active
4. `/rgb_left`, `/rgb_right` are the stereo camera topic and `/tf_right_wrist` is the tf from the world frame to the right_wrist_roll link frame
5. IK Pose is published to: `/hand_pose_ik`

---

## 🎥 Demo

![CLIK Demo](docs/vr_Issac_Sim_controller.webp)

## Citation
```
@article{cheng2024tv,
title={Open-TeleVision: Teleoperation with Immersive Active Visual Feedback},
author={Cheng, Xuxin and Li, Jialong and Yang, Shiqi and Yang, Ge and Wang, Xiaolong},
journal={arXiv preprint arXiv:2407.01512},
year={2024}
}
```
