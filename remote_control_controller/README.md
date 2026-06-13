# remote_control_controller

This ROS 2 package provides real-time teleoperation for the **Addverb Cobot** using a **Cosmic Byte Ares Gamepad** (or any standard XInput-compliant gamepad).

---

## 🎮 Gamepad Mapping Layout

### 🛠️ Control Modes
Toggle between control modes using the **X Button**.

#### **Mode 1: Cartesian Jogging (Default)**
Used for translating and rotating the Tool Center Point (TCP).
* **Left Analog Stick Y**: Move TCP forward/backward ($\pm$ X)
* **Left Analog Stick X**: Move TCP left/right ($\pm$ Y)
* **Right Analog Stick Y**: Move TCP up/down ($\pm$ Z)
* **Right Analog Stick X**: Rotate TCP (Yaw $\pm$ Z)
* **D-pad Y**: Rotate TCP (Pitch $\pm$ Y)
* **D-pad X**: Rotate TCP (Roll $\pm$ X)

#### **Mode 2: Joint Jogging**
Used for rotating individual joints (Joints 1 to 6).
* **Left Analog Stick Y**: Jog Joint 1
* **Left Analog Stick X**: Jog Joint 2
* **Right Analog Stick Y**: Jog Joint 3
* **Right Analog Stick X**: Jog Joint 4
* **D-pad Y**: Jog Joint 5
* **D-pad X**: Jog Joint 6

---

### 🛡️ Safety & Commands

* **LB (Left Bumper) [DEADMAN SWITCH]**: **Must be held down** to enable robot movement. Releasing LB instantly stops all jogging motion.
* **RB (Right Bumper) [SPEED TOGGLE]**: Press once to toggle between **SLOW** (precise) and **FAST** (coarse) movement speeds.
* **A Button**: Close Gripper
* **B Button**: Open Gripper
* **Y Button**: Trigger **Error Recovery** service (resets cobot errors)
* **Back Button**: Trigger **Safe Shutdown** service

---

## 🚀 Installation & Build Instructions

### Prerequisites
Make sure the ROS 2 joystick drivers package is installed:
```bash
sudo apt-get update
sudo apt-get install ros-humble-joy
```

### Build Steps
1. Navigate to your ROS 2 workspace's source directory:
   ```bash
   cd ~/cobot_ros2_ws/src
   ```
2. Create a symlink to the package directory:
   ```bash
   ln -s ~/remote_control_controller .
   ```
3. Build the package:
   ```bash
   cd ~/cobot_ros2_ws
   colcon build --packages-select remote_control_controller --symlink-install
   ```
4. Source the setup script:
   ```bash
   source install/setup.bash
   ```

---

## 🕹️ Running the Controller

1. Ensure the Cosmic Byte Ares wireless USB dongle is plugged into your PC, and the gamepad is turned on.
2. Verify the gamepad is registered by Linux (typically `/dev/input/js0`):
   ```bash
   ls -la /dev/input/js0
   ```
   *Note: If you get permission errors, run `sudo chmod a+rw /dev/input/js0`.*
3. Launch the gamepad controller:
   ```bash
   ros2 launch remote_control_controller gamepad_control.launch.py
   ```
4. Switch the cobot to `cartesian_jogging_controller` or `joint_jogging_controller` as desired (as detailed in the cobot control guides).
