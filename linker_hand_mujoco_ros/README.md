# 1. **Overview**

LinkerHand — skillful hands, creating everything.

The LinkerHand dexterous hand ROS SDK is a software tool developed by LinkerHand (Beijing) Technology Co., Ltd. It is used to drive its dexterous hand product line and provides functional examples. It supports multiple devices (such as laptops, desktops, Raspberry Pi, Jetson, etc.), mainly serving fields such as humanoid robots, industrial automation, and research institutes. It is suitable for scenarios such as humanoid robots, flexible production lines, embodied large-model training, and data collection.

# 1.1 **Notes**
This program provides a Mujoco simulation environment for the LinkerHand series of dexterous hands, making it convenient for users to become familiar with how to use LinkerHand dexterous hand products, and to perform model training and data collection in a simulation environment.

# 2. **Usage Instructions**
```bash
$ mkdir -p Linker_Hand_Mujoco_ros/src    # create directory
$ cd Linker_Hand_Mujoco_ros/src    # enter directory
$ # 1. Clone the repository (use sparse mode + blob filtering to save space)
$ git clone --filter=blob:none --sparse https://github.com/linkerbotai/linker_hand_sim.git
$ # 2. Enter the repository directory
$ cd linker_hand_sim
$ # 3. Set the sparse-checkout directory
$ git sparse-checkout set linker_hand_mujoco_ros
$ cd Linker_Hand_Mujoco_ros/src/linker_hand_sim/
$ pip install -r requirements.txt
```
- Modify linker_hand_mujoco_ros/launch/linker_hand_mujoco.launch  
Just edit according to the parameter descriptions in the file.
```bash
$ cd Linker_Hand_Mujoco_ros/
$ catkin_make
$ source ./devel/setup.bash
$ roslaunch linker_hand_mujoco_ros linker_hand_mujoco.launch
```

# 3. **Topic Description**
- /cb_right_hand_control_cmd or /cb_left_hand_control_cmd
```bash
rostopic pub /cb_right_hand_control_cmd sensor_msgs/JointState "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
name: ['']
position: [200,200,200,200,200,200,200,200,200,200]
velocity: [0]
effort: [0]"
```
- position description
  L7:  ["Thumb flexion", "Thumb roll (lateral swing)", "Index finger flexion", "Middle finger flexion", "Ring finger flexion", "Little finger flexion", "Thumb rotation"]

  L10: ["Thumb base", "Thumb lateral swing", "Index base", "Middle base", "Ring base", "Little finger base", "Index lateral swing", "Ring lateral swing", "Little finger lateral swing", "Thumb rotation"]

  L20: ["Thumb base", "Index base", "Middle base", "Ring base", "Little finger base", "Thumb lateral swing", "Index lateral swing", "Middle lateral swing", "Ring lateral swing", "Little finger lateral swing", "Thumb roll", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb tip segment", "Index distal end", "Middle distal end", "Ring distal end", "Little finger distal end"]

  L21: ["Thumb base", "Index base", "Middle base", "Ring base", "Little finger base", "Thumb lateral swing", "Index lateral swing", "Middle lateral swing", "Ring lateral swing", "Little finger lateral swing", "Thumb roll", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb middle segment", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb fingertip", "Index fingertip", "Middle fingertip", "Ring fingertip", "Little finger fingertip"]

  L25: ["Thumb base", "Index base", "Middle base", "Ring base", "Little finger base", "Thumb lateral swing", "Index lateral swing", "Middle lateral swing", "Ring lateral swing", "Little finger lateral swing", "Thumb roll", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb middle segment", "Index middle segment", "Middle middle segment", "Ring middle segment", "Little finger middle segment", "Thumb fingertip", "Index fingertip", "Middle fingertip", "Ring fingertip", "Little finger fingertip"]

# 3.1 **GUI Control**
You can use gui_control in Linker_Hand_SDK (https://github.com/linkerbotai/linker_hand_sdk/blob/main/README_CN.md) to control the simulation environment.
