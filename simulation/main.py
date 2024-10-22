import time
import numpy as np
import pickle
import sys
import os
import pybullet as p
from stretch import *

p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
p.setGravity(0, 0, -1.81)

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


mobot = init_scene(p)
    
forward=0
turn=0
speed=10
up=0
stretch=0
gripper_open=0
roll=0
yaw=0

mobot.get_observation()

while (1):
    time.sleep(1./240.)
    keys = p.getKeyboardEvents()

    for k,v in keys.items():
        # moving
        if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
            turn = -1
        if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_RELEASED)):
            turn = 0
        if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
            turn = 1
        if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_RELEASED)):
            turn = 0
        if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_TRIGGERED)):
            forward=1
        if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_RELEASED)):
            forward=0
        if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_TRIGGERED)):
            forward=-1
        if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_RELEASED)):
            forward=0

        # lifting
        if (k == ord('z') and (v & p.KEY_WAS_TRIGGERED)):
            up = 1
        if (k == ord('z') and (v & p.KEY_WAS_RELEASED)):
            up = 0
        if (k == ord('x') and (v & p.KEY_WAS_TRIGGERED)):
            up = -1
        if (k == ord('x') and (v & p.KEY_WAS_RELEASED)):
            up = 0

        # stretching
        if (k == ord('a') and (v & p.KEY_WAS_TRIGGERED)):
            stretch = -1
        if (k == ord('a') and (v & p.KEY_WAS_RELEASED)):
            stretch = 0
        if (k == ord('d') and (v & p.KEY_WAS_TRIGGERED)):
            stretch = 1
        if (k == ord('d') and (v & p.KEY_WAS_RELEASED)):
            stretch = 0

        # roll
        if (k == ord('r') and (v & p.KEY_WAS_TRIGGERED)):
            roll = 1
        if (k == ord('r') and (v & p.KEY_WAS_RELEASED)):
            roll = 0
        if (k == ord('f') and (v & p.KEY_WAS_TRIGGERED)):
            roll = -1
        if (k == ord('f') and (v & p.KEY_WAS_RELEASED)):
            roll = 0

        # yaw
        if (k == ord('y') and (v & p.KEY_WAS_TRIGGERED)):
            yaw = 1
        if (k == ord('y') and (v & p.KEY_WAS_RELEASED)):
            yaw = 0
        if (k == ord('h') and (v & p.KEY_WAS_TRIGGERED)):
            yaw = -1
        if (k == ord('h') and (v & p.KEY_WAS_RELEASED)):
            yaw = 0


        # gripper
        if (k == ord('q') and (v & p.KEY_WAS_TRIGGERED)):
            gripper_open = -1
        if (k == ord('q') and (v & p.KEY_WAS_RELEASED)):
            gripper_open = 0
        if (k == ord('e') and (v & p.KEY_WAS_TRIGGERED)):
            gripper_open = 1
        if (k == ord('e') and (v & p.KEY_WAS_RELEASED)):
            gripper_open = 0

    base_control(mobot, p, forward, turn)
    arm_control(mobot, p, up, stretch, roll, yaw)
    gripper_control(mobot, p, gripper_open)
   
    mobot.get_observation()

