# 1. **Overview**

LinkerHand — skillful hands, creating everything.

The LinkerHand dexterous hand ROS2 SDK is a software toolkit developed by LinkerHand (Beijing) Technology Co., Ltd. It is used to drive the company’s dexterous hand product line and provides functional examples. It supports multiple devices (such as laptops, desktops, Raspberry Pi, Jetson, etc.), primarily serving fields such as humanoid robots, industrial automation, and research institutions. It is suitable for scenarios such as humanoid robots, flexible production lines, embodied large-model training, and data collection.

# 1.1 **Notes**
This program provides a Mujoco simulation environment for the LinkerHand series of dexterous hands. It helps users become familiar with how to use LinkerHand dexterous hand products, and enables model training and data collection within a simulation environment.

# 2. **Usage**
```bash
$ mkdir -p linker_hand_pybullet_ros2/src    # create directory
$ cd linker_hand_pybullet_ros2/src    # enter directory
$ # 1. Clone the repository (use sparse mode + blob filtering to save space)
$ git clone --filter=blob:none --sparse https://github.com/linker-bot/linkerhand-sim.git
$ # 2. Enter the repository directory
$ cd linker_hand_sim
$ # 3. Set sparse-checkout directory
$ git sparse-checkout set linker_hand_pybullet_ros2
$ cd linker_hand_pybullet_ros2/src/linker_hand_sim/
$ pip install -r requirements.txt
```

- Modify linker_hand_pybullet_ros2/launch/linker_hand_pybullet_ros2.launch.py
Just edit according to the parameter descriptions in the file.
```bash
$ cd linker_hand_pybullet_ros2/
$ colcon build --symlink-install
$ source ./install/setup.bash
$ ros2 launch linker_hand_pybullet_ros2 linker_hand_pybullet_ros2.launch.py
```

# 3. **Topic Description**
- /cb_right_hand_control_cmd or /cb_left_hand_control_cmd
```bash
ros2 topic pub /joint_states sensor_msgs/msg/JointState '
{
  header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "" },
  name: [],
  position: [200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0],
  velocity: [],
  effort: []
}'

```

- position description
  L6:  ["Thumb flexion", "Thumb abduction/adduction (side swing)", "Index finger flexion", "Middle finger flexion", "Ring finger flexion", "Little finger flexion"]

  L7:  ["Thumb flexion", "Thumb abduction/adduction (side swing)", "Index finger flexion", "Middle finger flexion", "Ring finger flexion", "Little finger flexion", "Thumb rotation"]

  L10: ["Thumb base", "Thumb abduction/adduction (side swing)", "Index base", "Middle base", "Ring base", "Little finger base", "Index lateral swing", "Ring lateral swing", "Little finger lateral swing", "Thumb rotation"]

  L20: ["Thumb base", "Index base", "Middle base", "Ring base", "Little finger base", "Thumb lateral swing", "Index lateral swing", "Middle lateral swing", "Ring lateral swing", "Little finger lateral swing", "Thumb roll", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb tip segment", "Index distal end", "Middle distal end", "Ring distal end", "Little finger distal end"]

  L21: ["Thumb base", "Index base", "Middle base", "Ring base", "Little finger base", "Thumb lateral swing", "Index lateral swing", "Middle lateral swing", "Ring lateral swing", "Little finger lateral swing", "Thumb roll", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb middle segment", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb fingertip", "Index fingertip", "Middle fingertip", "Ring fingertip", "Little finger fingertip"]

  L25: ["Thumb base", "Index base", "Middle base", "Ring base", "Little finger base", "Thumb lateral swing", "Index lateral swing", "Middle lateral swing", "Ring lateral swing", "Little finger lateral swing", "Thumb roll", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb middle segment", "Index middle segment", "Middle middle segment", "Ring middle segment", "Little finger middle segment", "Thumb fingertip", "Index fingertip", "Middle fingertip", "Ring fingertip", "Little finger fingertip"]

# 3.1 **GUI Control**
You can use the gui_control in linker_hand_ros2_sdk (https://github.com/linkerbotai/linker_hand_ros2_sdk/blob/main/README_CN.md) to control the simulation environment.
