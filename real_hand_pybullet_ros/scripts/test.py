import pybullet as p
import pybullet_data
import time
path = "/home/hjx/ROS/real_hand_pybullet/src/real_hand_pybullet/urdf/real_hand_l20_8_left.urdf"
# Connect to the simulation
physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# Load the robot URDF
robot_id = p.loadURDF(path, basePosition=[0, 0, 0.1], useFixedBase=True)
plane_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[10, 10, 0.1])
plane_id = p.createMultiBody(0, plane_collision_shape)
p.setPhysicsEngineParameter(enableFileCaching=0)

p.setGravity(0, 0, -9.81)
time_step = 1.0 / 240.0
p.setTimeStep(time_step)

# Get the total number of joints and their info
num_joints = p.getNumJoints(robot_id)
print(f"Total number of robot joints: {num_joints}")

# Simulation loop
try:
    while True:
        p.stepSimulation()
        time.sleep(time_step)

        # Iterate over all joints and retrieve data
        for joint_index in range(num_joints):
            joint_state = p.getJointState(robot_id, joint_index)
            joint_position = joint_state[0]
            joint_velocity = joint_state[1]
            motor_torque = joint_state[3]

            print(f"Joint {joint_index}: position={joint_position:.3f}, velocity={joint_velocity:.3f}, torque={motor_torque:.3f}")

except KeyboardInterrupt:
    print("Simulation ended")
    p.disconnect()
