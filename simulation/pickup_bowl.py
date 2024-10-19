import time
import numpy as np
import os
import pybullet as p
from stretch import Robot

p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

p.setGravity(0, 0, -9.81)  # Set gravity

def initAxis(center, quater):
    rot_mat = p.getMatrixFromQuaternion(quater)
    rotmat = np.array(rot_mat).reshape((3, 3))
    p.addUserDebugLine(lineFromXYZ=center, lineToXYZ=center + rotmat[:3, 0] * 0.1, lineColorRGB=[1, 0, 0], lineWidth=10)
    p.addUserDebugLine(lineFromXYZ=center, lineToXYZ=center + rotmat[:3, 1] * 0.1, lineColorRGB=[0, 1, 0], lineWidth=10)
    p.addUserDebugLine(lineFromXYZ=center, lineToXYZ=center + rotmat[:3, 2] * 0.1, lineColorRGB=[0, 0, 1], lineWidth=10)


root_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../")

################ Plane Environment ################
plane_id = p.loadURDF(os.path.join(root_dir, "resource/urdf/plane.urdf"), [0, 0, 0])
p.changeVisualShape(plane_id, -1)

################ Load the robot ################
mobot_urdf_file = os.path.join(root_dir, "resource/urdf/stretch/stretch.urdf")
mobot = Robot(pybullet_api=p, start_pos=[-0.8, 0.0, 0.05], urdf_file=mobot_urdf_file)

# Let the simulation settle
for _ in range(30):
    p.stepSimulation()

################ Load the object to be lifted (e.g., a bowl) ################
bowl_position = [-0.7, -0.15, 0.3]  # Adjust based on the scene
bowl_orientation = p.getQuaternionFromEuler([0, 0, 0])
bowl_scaling = 0.1
bowl_id = p.loadURDF(os.path.join(root_dir, "resource/urdf/obj_libs/bowls/b1/model.urdf"),
                     basePosition=bowl_position,
                     baseOrientation=bowl_orientation,
                     globalScaling=bowl_scaling,
                     useFixedBase=False)

################ Movement Helper ################
def move_joint(robot, joint_index, target_pos, step_size=0.01, max_steps=500):
    """
    Moves the specified joint of the robot to the target position in small increments.
    """
    current_pos = p.getJointState(robot.robotId, joint_index)[0]
    step = 0
    while abs(current_pos - target_pos) > step_size and step < max_steps:
        new_pos = current_pos + step_size * np.sign(target_pos - current_pos)
        p.setJointMotorControl2(robot.robotId, joint_index, p.POSITION_CONTROL, targetPosition=new_pos)
        p.stepSimulation()
        current_pos = p.getJointState(robot.robotId, joint_index)[0]
        step += 1
        time.sleep(0.01)
    print(f"Joint {joint_index} reached target position {target_pos}")

################ Robot Control ################
# Example of controlling arm and gripper (Assuming the robot has specific joint indexes for these)
ARM_JOINT_INDEX = 4  # This should be set to the actual joint index of the robot's arm
GRIPPER_JOINT_INDEX = 6  # Replace with the actual index for the gripper

# Move arm down to the bowl position (assume a certain joint value corresponds to lowering the arm)
print("Moving arm to bowl position...")
move_joint(mobot, ARM_JOINT_INDEX, target_pos=0.1)  # Replace `0.3` with a suitable value for your arm

# Close gripper to grasp the object (gripper index might need fine-tuning)
print("Closing gripper to grasp the object...")
move_joint(mobot, GRIPPER_JOINT_INDEX, target_pos=0.02)  # Replace with actual gripper closed position

# Lift the arm after grasping
print("Lifting the object...")
move_joint(mobot, ARM_JOINT_INDEX, target_pos=0.6)  # Lift the arm to a higher position

# Open the gripper to release the object (optional)
print("Opening the gripper to release the object...")
move_joint(mobot, GRIPPER_JOINT_INDEX, target_pos=0.05)  # Replace with actual gripper open position

################ Main Simulation Loop ################
# Continue simulation for debugging and watching behavior
while True:
    p.stepSimulation()
    time.sleep(1 / 240.0)
