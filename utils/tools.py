import numpy as np
import pybullet as p

def get_robot_base_pose(p, robot_id):
    # base_link_index
    link_index = 4
    link_state = p.getLinkState(robot_id, link_index)
    link_position = link_state[0]
    link_orientation = link_state[1]
    print("Link Position: ", link_position)
    print("Link Orientation (quaternion): ", link_orientation)

    euler_orientation = p.getEulerFromQuaternion(link_orientation)
    print("Link Orientation (Euler angles): ", euler_orientation)

def get_joint_index_by_name(robot, joint_name):
    num_joints = p.getNumJoints(robot)
    
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot, i)
        if joint_info[1].decode("utf-8") == joint_name:
            print(f"joint name: {joint_name} found at index {i}")
            return i
    return None  # Return None if the joint name is not found

def get_link_index_by_name(robot, link_name):
    num_joints = p.getNumJoints(robot)  # This gives the number of joints, which is one less than the number of links
    
    for i in range(num_joints):
        link_info = p.getJointInfo(robot, i)
        link_name_in_urdf = link_info[12].decode("utf-8")  # Link name is stored at index 12
        if link_name_in_urdf == link_name:
            print(f"link name: {link_name_in_urdf} found at index {i}")
            return i
    return None  # Return None if the link name is not found

def getLinkInfo(object_id):
    numJoint = p.getNumJoints(object_id)
    LinkList = ['base']
    for jointIndex in range(numJoint):
      jointInfo = p.getJointInfo(object_id, jointIndex)
      link_name = jointInfo[12]
      if link_name not in LinkList:
        LinkList.append(link_name)
    return LinkList

def getNumLinks(object_id):
    return len(getLinkInfo(object_id))

def getAABB(object_id):
    numLinks = getNumLinks(object_id)
    AABB_List = []
    for link_id in range(-1, numLinks - 1):
        AABB_List.append(p.getAABB(object_id, link_id))
    AABB_array = np.array(AABB_List)
    AABB_obj_min = np.min(AABB_array[:, 0, :], axis=0)
    AABB_obj_max = np.max(AABB_array[:, 1, :], axis=0)
    AABB_obj = np.array([AABB_obj_min, AABB_obj_max])
    
    return AABB_obj