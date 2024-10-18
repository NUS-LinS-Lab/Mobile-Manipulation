import math
import os
import argparse
import sys
import time
import pybullet as p

import numpy as np


sys.path.append('./')

def initAxis(center, quater):
    rot_mat = p.getMatrixFromQuaternion(quater)
    rotmat = np.array(rot_mat).reshape((3, 3))
    p.addUserDebugLine(lineFromXYZ=center, lineToXYZ=center + rotmat[:3, 0] * 0.1, lineColorRGB=[1, 0, 0],
                            lineWidth=10)
    p.addUserDebugLine(lineFromXYZ=center, lineToXYZ=center + rotmat[:3, 1] * 0.1, lineColorRGB=[0, 1, 0],
                            lineWidth=10)
    p.addUserDebugLine(lineFromXYZ=center, lineToXYZ=center + rotmat[:3, 2] * 0.1, lineColorRGB=[0, 0, 1],
                            lineWidth=10)


class Robot:
    def __init__(self,pybullet_api,start_pos=[0.4,0.3,0.4],urdf_file=None,resource_dir=None,project_root_dir=None):
       self.p = p#pybullet_api

       self.gripperMaxForce = 1000.0
       self.armMaxForce = 200.0

       self.start_pos = start_pos
       self.camera_index = 18

       self.project_dir = project_root_dir
       self.resource_dir = resource_dir
       self.urdf_file = urdf_file
       self.robotId = self.p.loadURDF(self.urdf_file, self.start_pos, useFixedBase=True)
       self.p.resetBasePositionAndOrientation(self.robotId, self.start_pos, [0, 0, 0, 1])
       self.p.resetJointState(self.robotId, self.camera_index, -0.3)
       self.p.resetJointState(self.robotId, 4, 0.5)


    def get_observation(self):
       camera_link_pos = self.p.getLinkState(self.robotId,self.camera_index)[0]
       camera_link_ori = self.p.getLinkState(self.robotId,self.camera_index)[1]
       camera_link_rotmat = self.p.getMatrixFromQuaternion(camera_link_ori)
       camera_link_rotmat = np.array(camera_link_rotmat).reshape((3, 3))
       camera_target_link_pos = np.array(camera_link_pos)
       camera_target_link_pos = camera_target_link_pos + camera_link_rotmat[:,0]
       camera_color = [0, 0 / 255.0, 0 / 255.0, 1.0]
       self.p.changeVisualShape(self.robotId,self.camera_index,rgbaColor=[0,0,1])

       camera_view_matrix = self.p.computeViewMatrix(cameraEyePosition=[camera_link_pos[0], camera_link_pos[1], camera_link_pos[2]],
                                         cameraTargetPosition=[camera_target_link_pos[0], camera_target_link_pos[1], camera_target_link_pos[2]],
                                         cameraUpVector=camera_link_rotmat[:,1])


       ratio = 1.5
       image_width = int(640 * ratio)
       image_height = 480
       #self.p.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, camera_target_position)
       camera_proj_matrix = self.p.computeProjectionMatrixFOV(fov=45.0, aspect=1.0, nearVal=0.1, farVal=10)
       #$initAxis(camera_link_pos, camera_link_ori)
       self.p.getCameraImage(width=image_width,
                                      height=image_height,
                                      viewMatrix = camera_view_matrix,
                                      projectionMatrix=camera_proj_matrix,
                                      renderer = p.ER_BULLET_HARDWARE_OPENGL)
