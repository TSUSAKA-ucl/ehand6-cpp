# Unofficial HITBOT EHand-6 C++ Driver and ROS2 Package

This repository provides a C++ driver and ROS2 package for controlling
[the HITBOT eHand-6 robotic hand](https://www.hitbotrobot.com/cheap-dexterous-hand/).
The driver allows you to send commands to the eHand-6 over a CAN FD interface,
while the ROS2 package enables integration with ROS2-based robotic systems.

Programs are **independently created** with reference to the 
`EHand-6 Product User Manual` PDF in UC Lab. Nagoya University
and are **not official**.

## Directories
* `driver/` : C++ driver for eHand6, and demo program
* `ehand6_ctrl/` : ros2 package for eHand6 control, and demo program
* `ehand6_msgs/` : ros2 message definition for eHand6 control

## How to use
### control eHand-6 with C++ directly
0. Connect eHand-6 to CAN FD hardware and 24V3A power supply  
   If the socket CAN interface is not `can0`, please change it accordingly in the following steps.
1. Clone this repository
   ```
   git clone https://github.com/TSUSAKA-ucl/ehand6-cpp.git
   cd ehand6-cpp/driver/
   ```
2. Build
   ```
   make
   ```
3. Initialize CAN to CAN FD and set eHand6 bps
   ```
   ./can-up.sh
   ```
   Check
   ```
   ip -details link show can0
   ```
   It should show `bitrate 1000000 sample-point 0.800` and
   `dbitrate 5000000 dsample-point 0.750`.
   `can-up.sh` sets all socket CAN interfaces. If you want to set only a specific interface, use:
   ```
   source can-up.sh
   can_up can0
   ```
4. Turn on the 24V power supply
5. Test operation (for right hand on CAN interface `can0`)
   ```
   ./ehand_demo can0 right move reset
   ```
6. Test operation (for left hand on CAN interface `can1`)
   ```
   ./ehand_demo can1 left move reset
   ```
7. After the first time, zeroing is not required
   ```
   ./ehand_demo can1 left move
   ```
   To read the state only
   ```
   ./ehand_demo can1 left
   ```
   To move according to a YAML file
   ```
   ./ehand_demo can0 right sample.yaml
   ```
### Launch ROS2 node (for right hand on CAN interface `can0`) and control with joint_state_publisher_gui
1. Create a ROS2 workspace and clone this repository
   ```
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/TSUSAKA-ucl/ehand6-cpp.git
   ```
2. Build the ROS2 package
   ```
   colcon build
   ```
3. Build `ehand6_demo` executable
   ```
   cd ehand6-cpp/driver/
   make
   cd -
   ```
4. Source the ROS2 workspace
   ```
   source ~/ros2_ws/install/setup.bash
   ```
5. Zeroing the hand (for right hand on CAN interface `can0`)
   ```
   ./driver/ehand_demo can0 right
   ```
   for left hand on CAN interface `can1`
   ```
   ./driver/ehand_demo can1 left
   ```
6. Launch the ROS2 node and control with `joint_state_publisher_gui`  
   for right hand on CAN interface `can0`
   ```
   ros2 launch ehand6_ctrl ehand6_test.launch.py can_interface:=can0 hand_side:=right
   ```
   for left hand on CAN interface `can1`
   ```
   ros2 launch ehand6_ctrl ehand6_test.launch.py can_interface:=can1 hand_side:=left
   ```
