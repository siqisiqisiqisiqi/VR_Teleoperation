#!/bin/bash

# Activate your virtualenv
# echo "[INFO] Activating pyenv virtualenv..."
# eval "$(pyenv init -)"
# pyenv activate pinocchio-env

# Navigate to your workspace source folder
cd ~/vr_ros2_ws/src/data_collection

# # Launch each script in the background
# echo "[INFO] Running initial_pose..."
# python3 ../pinocchio_ik_solver/pinocchio_ik_solver/initial_pose_publisher.py &

# sleep 5  # Optional: let initial_pose finish

# echo "[INFO] Running GUI slider..."
# python3 ../pinocchio_ik_solver/pinocchio_ik_solver/slider_gui_pose.py &

echo "[INFO] Keyboard control..."
python3 data_collection/keyboard_pose_teleop.py &

echo "[INFO] Running IK solver..."
python3 ../pinocchio_ik_solver/pinocchio_ik_solver/clik_keyboard_ros.py &

# echo "[INFO] Running data collector..."
# python3 data_collection/data_collection.py &

# Wait for all background jobs to finish
wait
