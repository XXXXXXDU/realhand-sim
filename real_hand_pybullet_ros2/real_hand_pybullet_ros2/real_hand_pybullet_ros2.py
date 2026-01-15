#!/usr/bin/env python3
import rclpy, os, threading, time
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from importlib.resources import files

from realhand_sim_shared.utils.mapping import *
L6_MAPPING = {
    0: 1,
    1: 0,
    2: 0,
    3: 2,
    4: 2,
    5: 3,
    6: 3,
    7: 4,
    8: 4,
    9: 5,
    10: 5,
    
}
L7_MAPPING = {
    0: (6, 1),
    1: (1, 1),
    2: (0, 1),
    3: (0, 3),
    4: (0, 2),
    5: (0, 2),
    7: (2, 1),
    8: (2, 1),
    9: (2, 1),
    11: (3, 1),
    12: (3, 1),
    13: (3, 1),
    16: (4, 1),
    17: (4, 1),
    18: (4, 1),
    21: (5, 1),
    22: (5, 1),
    23: (5, 1)
}
L10_MAPPING = {
    0: 9,    # Special joint
    1: 1,    # Thumb
    2: 0,    # Index finger base joint
    3: 0,    # Index finger middle joint
    4: 0,    # Index finger distal joint
    6: 6,    # Special joint
    7: 2,    # Middle finger base joint
    8: 2,    # Middle finger middle joint
    9: 2,    # Middle finger distal joint
    11: 3,   # Ring finger base joint
    12: 3,   # Ring finger middle joint
    13: 3,   # Ring finger distal joint
    15: 7,   # Special joint
    16: 4,   # Little finger base joint
    17: 4,   # Little finger middle joint
    18: 4,   # Little finger distal joint
    20: 8,   # Special joint
    21: 5,   # Extra joint 1
    22: 5,   # Extra joint 2
    23: 5    # Extra joint 3
}
L20_MAPPING = {
    0: 0, 7: 1, 12: 2, 17: 3, 22: 4,
    1: 5, 6: 6, 11: 7, 16: 8, 21: 9,
    2: 10, 3: 15, 8: 16, 13: 17, 18: 18, 23: 19
}
L21_MAPPING = {
    0: 6,  1: 1,   2: 21,   
    3: 7,  4: 2,  5: 22,
    6: 8, 7: 3,   8: 23,
    9: 9,  10: 4, 11: 24,
    12: 10, 13: 5, 
    14: 0,  15: 15, 16: 20
}
class RealHandPyBulletNode(Node):
    def __init__(self):
        super().__init__('real_hand_pybullet_ros2_node')
        self.declare_parameter('hand_joint', 'L7')
        #self.hand_type = "right"
        self.hand_joint = self.get_parameter('hand_joint').value
        self.left_hand_state_pub = self.create_publisher(JointState, '/cb_left_hand_state_sim', 10)
        self.right_hand_state_pub = self.create_publisher(JointState, '/cb_right_hand_state_sim', 10)
        self.create_subscription(JointState,f"/cb_left_hand_control_cmd",self.left_hand_cb,10)
        self.create_subscription(JointState,f"/cb_right_hand_control_cmd",self.right_hand_cb,10)
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.init_sim()
        self.pub_threading = threading.Thread(target=self.pub_hand_state, daemon=True)
        self.pub_threading.start()
        self.sim_threading = threading.Thread(target=self.sim_controller.showSim, daemon=True)
        self.sim_threading.start()
    

    def init_sim(self):
        hand = self.hand_joint.lower()
        urdf_path_left = str(files("realhand_sim_shared").joinpath("urdf", hand, "left", f"realhand_{hand}_left.urdf"))
        urdf_path_right = str(files("realhand_sim_shared").joinpath("urdf", hand, "right", f"realhand_{hand}_right.urdf"))
        
        if self.hand_joint == "L6":
            from realhand_sim_shared.utils.l6_sim_controller import L6SimController
            self.sim_controller = L6SimController(urdf_path_left=urdf_path_left, urdf_path_right=urdf_path_right)
        elif self.hand_joint == "L7":
            from realhand_sim_shared.utils.l7_sim_controller import L7SimController
            self.sim_controller = L7SimController(urdf_path_left=urdf_path_left, urdf_path_right=urdf_path_right)
        elif self.hand_joint == "L10":
            from realhand_sim_shared.utils.l10_sim_controller import L10SimController
            self.sim_controller = L10SimController(urdf_path_left=urdf_path_left, urdf_path_right=urdf_path_right)
        elif self.hand_joint == "L20":
            from realhand_sim_shared.utils.l20_sim_controller import L20SimController
            urdf_path_left = f"{self.current_dir}/urdf/{self.hand_joint.lower()}/real_hand_l20_left/real_hand_{self.hand_joint.lower()}_left.urdf"
            print(urdf_path_left)
            urdf_path_right = f"{self.current_dir}/urdf/{self.hand_joint.lower()}/real_hand_l20_right/real_hand_{self.hand_joint.lower()}_right.urdf"
            self.sim_controller = L20SimController(urdf_path_left=urdf_path_left, urdf_path_right=urdf_path_right)
        elif self.hand_joint == "L21":
            from realhand_sim_shared.utils.l21_sim_controller import L21SimController
            self.sim_controller = L21SimController(urdf_path_left=urdf_path_left, urdf_path_right=urdf_path_right)

        

    def pub_hand_state(self):
        tmp_pose = [0.00] * 25
        while True:
            if len(self.sim_controller.left_hand_current_position) == 0:
                #self.get_logger().info("Left hand position is empty, waiting for data...")
                pass
            else:
                # TODO: This returns the number of simulated joints and needs to be converted to the number of joints published on the topic
                left_pose = self.sim_controller.left_hand_current_position
                if self.hand_joint == "L21":
                    tmp_pose = [0.00] * 25
                    items = L21_MAPPING.items()
                    for target_idx, source_idx in items:
                        tmp_pose[source_idx] = left_pose[target_idx]
                    left_state_msg = self.joint_msg(hand="left",position=arc_to_range_left(hand_arc_l=tmp_pose,hand_joint=self.hand_joint), velocity=[], effort=[])
                    self.left_hand_state_pub.publish(left_state_msg)
                else:
                    left_state_msg = self.joint_msg(hand="left",position=arc_to_range_left(hand_arc_l=left_pose,hand_joint=self.hand_joint), velocity=[], effort=[])
                    self.left_hand_state_pub.publish(left_state_msg)
            if len(self.sim_controller.right_hand_current_position) == 0:
                #self.get_logger().info("Right hand position is empty, waiting for data...")
                pass
            else:
                right_pose = self.sim_controller.right_hand_current_position
                if self.hand_joint == "L21":
                    tmp_pose = [0.00] * 25
                    items = L21_MAPPING.items()
                    for target_idx, source_idx in items:
                        tmp_pose[source_idx] = right_pose[target_idx]
                    right_state_msg = self.joint_msg(hand="right",position=arc_to_range_right(right_arc=tmp_pose,hand_joint=self.hand_joint), velocity=[], effort=[])
                    self.right_hand_state_pub.publish(right_state_msg)
                else:
                    right_state_msg = self.joint_msg(hand="right",position=arc_to_range_right(right_arc=right_pose,hand_joint=self.hand_joint), velocity=[], effort=[])
                    self.right_hand_state_pub.publish(right_state_msg)
            time.sleep(0.1)

    def left_hand_cb(self, msg):
        left_hand_pos = [0] * 25
        if len(msg.position) > 0 :
            if self.hand_joint == "L6":
                cmd_left_pos = range_to_arc_left(left_range=list(msg.position),hand_joint="L6")
                for target_idx, source_idx in L6_MAPPING.items():
                    left_hand_pos[target_idx] = cmd_left_pos[source_idx]
                self.sim_controller.set_left_position(pos=left_hand_pos)
            elif self.hand_joint == "L7":
                cmd_left_pos = range_to_arc_left(left_range=list(msg.position),hand_joint="L7")
                for target_idx, (source_idx, multiplier) in L7_MAPPING.items():
                    left_hand_pos[target_idx] = cmd_left_pos[source_idx] * multiplier
                self.sim_controller.set_left_position(pos=left_hand_pos)
            elif self.hand_joint == "L10":
                cmd_left_pos = range_to_arc_left(left_range=list(msg.position),hand_joint="L10")
                for target_idx, source_idx in L10_MAPPING.items():
                    left_hand_pos[target_idx] = cmd_left_pos[source_idx]
                self.sim_controller.set_left_position(pos=left_hand_pos)
            elif self.hand_joint == "L20":
                cmd_left_pos = range_to_arc_left(left_range=list(msg.position),hand_joint="L20")
                for target_idx, source_idx in L20_MAPPING.items():
                    left_hand_pos[target_idx] = cmd_left_pos[source_idx]
                self.sim_controller.set_left_position(pos=left_hand_pos)
            elif self.hand_joint == "L21":
                cmd_left_pos = range_to_arc_left(left_range=list(msg.position),hand_joint="L21")
                for target_idx, source_idx in L21_MAPPING.items():
                    left_hand_pos[target_idx] = cmd_left_pos[source_idx]
                self.sim_controller.set_left_position(pos=left_hand_pos)
    
    def right_hand_cb(self, msg):
        right_hand_pos = [0] * 25
        if self.hand_joint == "L6":
            cmd_right_pos = range_to_arc_right(right_range=list(msg.position),hand_joint="L6")
            for target_idx, source_idx in L6_MAPPING.items():
                right_hand_pos[target_idx] = cmd_right_pos[source_idx]
            self.sim_controller.set_right_position(pos=right_hand_pos)
        elif self.hand_joint == "L7":
            cmd_right_pos = range_to_arc_right(right_range=list(msg.position),hand_joint="L7")
            for target_idx, (source_idx, multiplier) in L7_MAPPING.items():
                right_hand_pos[target_idx] = cmd_right_pos[source_idx] * multiplier
            self.sim_controller.set_right_position(pos=right_hand_pos)
        elif self.hand_joint == "L10":
            cmd_right_pos = range_to_arc_right(right_range=list(msg.position),hand_joint="L10")
            for target_idx, source_idx in L10_MAPPING.items():
                right_hand_pos[target_idx] = cmd_right_pos[source_idx]
            self.sim_controller.set_right_position(pos=right_hand_pos)
        elif self.hand_joint == "L20":
            cmd_right_pos = range_to_arc_right(right_range=list(msg.position),hand_joint="L20")
            for target_idx, source_idx in L20_MAPPING.items():
                right_hand_pos[target_idx] = cmd_right_pos[source_idx]
            self.sim_controller.set_right_position(pos=right_hand_pos)
        elif self.hand_joint == "L21":
                cmd_right_pos = range_to_arc_right(right_range=list(msg.position),hand_joint="L21")
                for target_idx, source_idx in L21_MAPPING.items():
                    right_hand_pos[target_idx] = cmd_right_pos[source_idx]
                self.sim_controller.set_right_position(pos=right_hand_pos)


    def joint_msg(self,hand,position,velocity,effort):
        # Initialize JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        if hand == "left":
            joint_state_msg.name = []  # Joint names
        elif hand == "right":
            joint_state_msg.name = []  # Joint names
        joint_state_msg.position = position  # Joint positions (radians)
        joint_state_msg.velocity = velocity  # Joint velocities
        joint_state_msg.effort = effort  # Joint torques
        return joint_state_msg

            
    # def timer_callback(self):
    #     msg = String()
    #     msg.data = 'Hello ROS 2!'
    #     self.publisher_.publish(msg)
    #     self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = RealHandPyBulletNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
