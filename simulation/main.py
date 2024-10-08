import time
import numpy as np
import pickle
import sys
import os
import pybullet as p
from stretch import Robot

p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

p.setGravity(0, 0, -1.81)

def initAxis(center, quater):
    rot_mat = p.getMatrixFromQuaternion(quater)
    rotmat = np.array(rot_mat).reshape((3, 3))
    p.addUserDebugLine(lineFromXYZ=center, lineToXYZ=center + rotmat[:3, 0] * 0.1, lineColorRGB=[1, 0, 0],
                            lineWidth=10)
    p.addUserDebugLine(lineFromXYZ=center, lineToXYZ=center + rotmat[:3, 1] * 0.1, lineColorRGB=[0, 1, 0],
                            lineWidth=10)
    p.addUserDebugLine(lineFromXYZ=center, lineToXYZ=center + rotmat[:3, 2] * 0.1, lineColorRGB=[0, 0, 1],
                            lineWidth=10)


root_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)),"../")

################ Plane Environment
plane_id = p.loadURDF(os.path.join(root_dir,"resource/urdf/plane.urdf"), [0, 0, 0])
plane_texture_id = p.loadTexture(os.path.join(root_dir,"resource/texture/texture1.jpg"))
p.changeVisualShape(0,-1,textureUniqueId=plane_texture_id)

################ Robot
mobot_urdf_file = os.path.join(root_dir,"resource/urdf/stretch/stretch.urdf")
mobot = Robot(pybullet_api=p, start_pos=[-0.8,0.0,0.05], urdf_file=mobot_urdf_file)

for _ in range(30):
    p.stepSimulation()


################
#### table initialization
table_height = 0.8
table_width = 1.10 * 2.0
table_depth = 1.0
table_v = p.createVisualShape(p.GEOM_BOX, halfExtents=[table_depth / 2.0, table_width / 2.0,
                                                                      table_height / 2.0])
table_c = p.createCollisionShape(p.GEOM_BOX, halfExtents=[table_depth / 2.0, table_width / 2.0,
                                                                         table_height / 2.0])
mass = 0
table_id = p.createMultiBody(mass, baseCollisionShapeIndex=table_c, baseVisualShapeIndex=table_v,
                                       basePosition=(table_depth / 2.0 + 0.1, 0.05, table_height / 2.0))
table_color = [128 / 255.0, 128 / 255.0, 128 / 255.0, 1.0]
p.changeVisualShape(table_id, -1, rgbaColor=table_color)

################
wall_height = 2.2
wall_width = table_width + 4.0
wall_depth = 0.02
wall_v = p.createVisualShape(p.GEOM_BOX, halfExtents=[wall_depth/2.0, wall_width/2.0, wall_height/2.0])
wall_c = p.createCollisionShape(p.GEOM_BOX, halfExtents=[wall_depth/2.0, wall_width/2.0, wall_height/2.0])
mass = 0
wall_center_x = p.getAABB(table_id)[1][0] + wall_depth/2.0

wall_v2 = p.createVisualShape(p.GEOM_BOX, halfExtents=[wall_depth/2.0, wall_width/2.0-0.5, wall_height/2.0])
wall_c2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[wall_depth/2.0, wall_width/2.0-0.5, wall_height/2.0])

wall_id = p.createMultiBody(mass,\
                                          baseCollisionShapeIndex=wall_c2,\
                                          baseVisualShapeIndex=wall_v2,\
                                          basePosition=(wall_center_x, -1.4, wall_height/2.0))

wall_id_back = p.createMultiBody(mass,\
                                          baseCollisionShapeIndex=wall_c,\
                                          baseVisualShapeIndex=wall_v,\
                                          basePosition=(wall_center_x-3.0, -1.9, wall_height/2.0))


wall_id_front = p.createMultiBody(mass,\
                                          baseCollisionShapeIndex=wall_c,\
                                          baseVisualShapeIndex=wall_v,\
                                          basePosition=(wall_center_x+3.0, -1.9, wall_height/2.0))

wall_width_left = 2.0
wall_width_right = 6.0
wall_depth = 0.02
wall_v_left = p.createVisualShape(p.GEOM_BOX,halfExtents=[wall_depth/2.0, wall_width_left/2.0, wall_height/2.0])
wall_c_left = p.createCollisionShape(p.GEOM_BOX,halfExtents=[wall_depth/2.0, wall_width_left/2.0, wall_height/2.0])
wall_v_right = p.createVisualShape(p.GEOM_BOX,halfExtents=[wall_depth/2.0, wall_width_right/2.0, wall_height/2.0])
wall_c_right = p.createCollisionShape(p.GEOM_BOX,halfExtents=[wall_depth/2.0, wall_width_right/2.0, wall_height/2.0])

mass = 0
wall_left_center_x = p.getAABB(table_id)[1][0] - wall_width_left/2.0
wall_right_center_x = wall_left_center_x
wall_left_center_y = p.getAABB(table_id)[0][1] - wall_depth/2.0
wall_right_center_y = p.getAABB(table_id)[1][1] + wall_depth/2.0

wall_left_id = p.createMultiBody(mass,\
                                          baseCollisionShapeIndex=wall_c_left,\
                                          baseVisualShapeIndex=wall_v_left,\
                                          basePosition=(wall_left_center_x-1.0, wall_left_center_y-0.92, wall_height/2.0),
                                          baseOrientation=p.getQuaternionFromEuler((0,0,np.pi/2.0)))
wall_right_id = p.createMultiBody(mass,\
                                          baseCollisionShapeIndex=wall_c_right,\
                                          baseVisualShapeIndex=wall_v_right,\
                                          basePosition=(wall_right_center_x+1.0, wall_right_center_y, wall_height/2.0),
                                          baseOrientation=p.getQuaternionFromEuler((0,0,-np.pi/2.0)))

wall_right_id2 = p.createMultiBody(mass,\
                                          baseCollisionShapeIndex=wall_c_right,\
                                          baseVisualShapeIndex=wall_v_right,\
                                          basePosition=(wall_right_center_x+1.0, wall_right_center_y-6.1, wall_height/2.0),
                                          baseOrientation=p.getQuaternionFromEuler((0,0,-np.pi/2.0)))

wall_color = [204/255.0,242/255.0,255/255.0,1.0]
p.changeVisualShape(wall_left_id,-1,rgbaColor=wall_color)
p.changeVisualShape(wall_right_id,-1,rgbaColor=wall_color)
p.changeVisualShape(wall_id,-1,rgbaColor=wall_color)

urdf_dir = os.path.join(root_dir,"resource/urdf")

table_z = p.getAABB(table_id)[1][2]

cabinet2_position = [-1.5, 0.25, table_z+ 1.5]
cabinet2_scaling = 0.7
cabinet2_orientation = p.getQuaternionFromEuler([0, 0, np.pi])
cabinet2_id = p.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/cabinets/c2/mobility.urdf"),\
                                   useFixedBase=True,
                                   basePosition=cabinet2_position,\
                                   baseOrientation=cabinet2_orientation,\
                                   globalScaling=cabinet2_scaling)

p.changeVisualShape(cabinet2_id,2,rgbaColor=[0.5,0.5,0.5,1])
p.changeVisualShape(cabinet2_id,1,rgbaColor=[1,1,1,1])
p.changeVisualShape(cabinet2_id,3,rgbaColor=[1,1,1,1])
p.changeVisualShape(cabinet2_id,4,rgbaColor=[0.5,0.5,0.5,1])


cabinet_center_x = 1.35 #+ (p.getAABB(table_id)[1][0] - p.getAABB(cabinet1_id)[1][0])/2.0
cabinet_center_y = -1.25#cabinet_width/2.0
cabinet_center_z = 1.4

#cabinet1_position = (cabinet_center_x, -cabinet_center_y, cabinet_center_z)
cabinet2_position = (cabinet_center_x,  cabinet_center_y, cabinet_center_z)
#p.resetBasePositionAndOrientation(cabinet1_id, cabinet1_position, cabinet1_orientation)
p.resetBasePositionAndOrientation(cabinet2_id, cabinet2_position, cabinet2_orientation)

############################
#### fridge initialization
fridge_position = [0.7, -3.22, 0.9]
fridge_scaling = 1.0
fridge_orientation = p.getQuaternionFromEuler([0, 0, 0])
fridge_id = p.loadURDF(fileName=os.path.join(urdf_dir, "obj_libs/fridges/f1/mobility.urdf"), \
                                 useFixedBase=True, \
                                 basePosition=fridge_position, \
                                 baseOrientation=fridge_orientation, \
                                 globalScaling=fridge_scaling)


#######
table_z = p.getAABB(table_id)[1][2]
drawer_position = [3.84, 0.05,  0.42]
drawer_scaling = 0.5
drawer_orientation = p.getQuaternionFromEuler([0, 0, 0])
drawer_id = p.loadURDF(os.path.join(urdf_dir, "obj_libs/drawers/d1/mobility.urdf"), \
                                 basePosition=drawer_position, \
                                 baseOrientation=drawer_orientation, \
                                 globalScaling=drawer_scaling, \
                                 useFixedBase=True)

#### bed
#### table initialization
bed_height = 0.7#0.12 * 2.0
bed_width = 1.8
bed_depth = 2.2
bed_v = p.createVisualShape(p.GEOM_BOX, halfExtents=[bed_depth / 2.0, bed_width / 2.0,
                                                                      bed_height / 2.0])
bed_c = p.createCollisionShape(p.GEOM_BOX, halfExtents=[bed_depth / 2.0, bed_width / 2.0,
                                                                         bed_height / 2.0])
mass = 0
bed_id = p.createMultiBody(mass, baseCollisionShapeIndex=bed_c, baseVisualShapeIndex=bed_v,
                                       basePosition=(bed_depth / 2.0 + 1.9, -1.45, bed_height / 2.0))
bed_color = [128 / 255.0, 128 / 255.0, 128 / 255.0, 1.0]
p.changeVisualShape(bed_id, -1, rgbaColor=bed_color)

#### microwave initialization
table_z = p.getAABB(table_id)[1][2]
microwave_position = [0.35, 0.72, table_z + 0.15]
microwave_scaling = 0.4
microwave_orientation = p.getQuaternionFromEuler([0, 0, 0])

microwave_id = p.loadURDF(os.path.join(urdf_dir, "obj_libs/microwaves/7128/mobility.urdf"), \
                                    basePosition=microwave_position, \
                                    baseOrientation=microwave_orientation, \
                                    globalScaling=microwave_scaling, \
                                    useFixedBase=True)

p.changeVisualShape(microwave_id, 1, rgbaColor=[0.2, 0.2, 0.2, 1], specularColor=[1., 1., 1.])
p.changeVisualShape(microwave_id, 0, rgbaColor=[0.4, 0.4, 0.4, 1], specularColor=[1., 1., 1.])
p.changeVisualShape(microwave_id, 2, rgbaColor=[0.5, 0.5, 0.5, 1])
p.changeVisualShape(microwave_id, 3, rgbaColor=[0.2, 0.2, 0.2, 1])
p.resetJointState(microwave_id, 1, np.pi/2.0, 0.0)
#####

box_position = [2.25, -3.5 , 0.2]
box_scaling = 0.4
box_orientation = p.getQuaternionFromEuler([0, 0.0, np.pi / 2.0 + 0])
box_id = p.loadURDF(os.path.join(urdf_dir, "obj_libs/boxes/b4/mobility.urdf"),
                              basePosition=box_position,
                              baseOrientation=box_orientation,
                              globalScaling=box_scaling,
                              useFixedBase=False,
                              flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL)

numJoint = p.getNumJoints(box_id)
box_AABB = p.getAABB(box_id, 0)
box_height = box_AABB[1][2] - box_AABB[0][2]
p.resetBasePositionAndOrientation(box_id, box_position, box_orientation)
bbox = p.getAABB(box_id)
bbox2 = p.getAABB(box_id, 0)
p.resetJointState(box_id, 1, 0.9, 0.0)
for ji in range(numJoint):
    p.setJointMotorControl2(box_id, ji, p.VELOCITY_CONTROL, force=0.5)

############################
bottle_position = [drawer_position[0]+0.1, drawer_position[1]+0.1, table_z+0.49]
bottle_scaling = 0.2
bottle_orientation = p.getQuaternionFromEuler([np.pi/2.0, 0.0, 0.0])
bottle_id = p.loadURDF(os.path.join(urdf_dir, "obj_libs/bottles/b3/mobility.urdf"),
                                  basePosition=bottle_position,
                                  baseOrientation=bottle_orientation,
                                  useFixedBase=False,
                                  globalScaling=bottle_scaling,
                                  flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL)

obj_friction_ceof = 2000.0
p.changeDynamics(bottle_id, -1, lateralFriction=obj_friction_ceof)
p.changeDynamics(bottle_id, -1, rollingFriction=obj_friction_ceof)
p.changeDynamics(bottle_id, -1, spinningFriction=obj_friction_ceof)

p.changeDynamics(bottle_id, -1, mass=0.02)
#p.changeDynamics(bottle_id, -1, linearDamping=20.0)
#p.changeDynamics(bottle_id, -1, angularDamping=20.0)
#p.changeDynamics(bottle_id, -1, contactStiffness=0.1, contactDamping=0.1)


bowl_position = [0.4, -0.6, table_z + 0.15]
bowl_scaling = 0.2
bowl_orientation = p.getQuaternionFromEuler([.0, 0.0, 0.0])
bowl_id = p.loadURDF(os.path.join(urdf_dir, "obj_libs/bowls/b1/model.urdf"), \
                               basePosition=bowl_position, \
                               baseOrientation=bowl_orientation, \
                               globalScaling=bowl_scaling, \
                               useFixedBase=False, \
                               flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL)
bowl_AABB = p.getAABB(bowl_id)
bowl_height = bowl_AABB[1][2] - bowl_AABB[0][2]
bowl_position[2] = table_z + bowl_height / 2.0
p.resetBasePositionAndOrientation(bowl_id, bowl_position, bowl_orientation)
obj_friction_ceof = 2000.0

p.changeDynamics(bowl_id, -1, lateralFriction=obj_friction_ceof)
p.changeDynamics(bowl_id, -1, rollingFriction=obj_friction_ceof)
p.changeDynamics(bowl_id, -1, spinningFriction=obj_friction_ceof)
p.changeDynamics(bowl_id, -1, mass=0.2)
#p.changeDynamics(self.bowl_id, -1, linearDamping=20.0)
#p.changeDynamics(self.bowl_id, -1, angularDamping=20.0)
#p.changeDynamics(self.bowl_id, -1, contactStiffness=0.9, contactDamping=0.9)

mug_position = [0.25, -0.93, 1.53]
mug_orientation = p.getQuaternionFromEuler([np.pi / 2.0, 0, np.pi - np.pi / 2.0])
mug_scaling = 0.2
mug_id = p.loadURDF(fileName=os.path.join(urdf_dir, "obj_libs/mugs/m2/model.urdf"),
                              useFixedBase=False,
                              globalScaling=mug_scaling,
                              basePosition=mug_position,
                              baseOrientation=mug_orientation)
# self.p.changeVisualShape(self.mug_id, -1, rgbaColor=[1.,1.,1.0,1])
obj_friction_ceof = 4000.0
p.changeDynamics(mug_id, -1, lateralFriction=obj_friction_ceof)
p.changeDynamics(mug_id, -1, rollingFriction=obj_friction_ceof)
p.changeDynamics(mug_id, -1, spinningFriction=obj_friction_ceof)
p.changeDynamics(mug_id, -1, mass=0.01)


trashbin_position = [-1.1, -4.01, 0.48]
trashbin_scaling = 1.0
trashbin_orientation = p.getQuaternionFromEuler([np.pi / 2.0, 0.0, np.pi / 2.0])
trashbin_id = p.loadURDF(fileName=os.path.join(urdf_dir, "obj_libs/trashbins/t2/model.urdf"), \
                                   useFixedBase=False,
                                   basePosition=trashbin_position, \
                                   baseOrientation=trashbin_orientation, \
                                   globalScaling=trashbin_scaling)
p.changeVisualShape(trashbin_id, -1, rgbaColor=[200 / 255., 179 / 255., 179 / 255., 1])

pan_position = [0.35, .2, table_z + 0.05]
pan_scaling = 0.6
pan_orientation = p.getQuaternionFromEuler([.0, 0.0, np.pi / 4.0])
pan_id = p.loadURDF(os.path.join(urdf_dir, "obj_libs/pans/p1/model.urdf"), \
                              basePosition=pan_position, \
                              baseOrientation=pan_orientation, \
                              globalScaling=pan_scaling, \
                              useFixedBase=False, \
                              flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL)
p.changeVisualShape(pan_id, 1, rgbaColor=[0.9, 0.9, 0.9, 1.0])
p.changeDynamics(pan_id, -1, mass=0.001)

spatula_position = np.copy(np.array(pan_position))
spatula_position[1] -= 0.3
spatula_position[0] += 0.25
spatula_position[2] += 0.1
spatula_scaling = 0.4
spatula_orientation = p.getQuaternionFromEuler([np.pi/2-np.pi/8.,0.,0.])
spatula_id = p.loadURDF( os.path.join(urdf_dir, "obj_libs/spatula/model.urdf"),\
                                       basePosition=spatula_position,\
                                       baseOrientation=spatula_orientation,\
                                       globalScaling=spatula_scaling,\
                                       flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL,useFixedBase=False)

p.changeVisualShape(spatula_id, -1, rgbaColor=[0 / 255.0, 179 / 255., 179 / 255., 1])
p.changeDynamics(spatula_id, -1, mass=0.01)


mug_position = [drawer_position[0], drawer_position[1], 2.3]
mug_orientation = p.getQuaternionFromEuler([np.pi/2.0, 0, np.pi + np.pi/2.0])
mug_scaling = 0.25
mug_id = p.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/mugs/m1/model.urdf"),
                                  useFixedBase=False,
                                  globalScaling=mug_scaling,
                                  basePosition=mug_position,
                                  baseOrientation=mug_orientation)
p.changeVisualShape(mug_id, -1, rgbaColor=[1.0,1.0,1.0,1])
obj_friction_ceof = 4000.0
p.changeDynamics(mug_id, -1, lateralFriction=obj_friction_ceof)
p.changeDynamics(mug_id, -1, mass=0.01)



for _ in range(20):
    p.stepSimulation()

p.changeVisualShape(mobot.robotId,0,rgbaColor=[1,0,0,1])
p.changeVisualShape(mobot.robotId,1,rgbaColor=[0,1,0,1])

p.setRealTimeSimulation(1)

for j in range (p.getNumJoints(mobot.robotId)):
    print(p.getJointInfo(mobot.robotId,j))
forward=0
turn=0
speed=10

for _ in range(30):
    p.stepSimulation()


mobot.get_observation()

while (1):
    time.sleep(1./240.)
    keys = p.getKeyboardEvents()
    leftWheelVelocity=0
    rightWheelVelocity=0

    for k,v in keys.items():

        if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                turn = -0.8
        if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_RELEASED)):
                turn = 0
        if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                turn = 0.8
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

    rightWheelVelocity+= (forward+turn)*speed
    leftWheelVelocity += (forward-turn)*speed
    
    p.setJointMotorControl2(mobot.robotId,0,p.VELOCITY_CONTROL,targetVelocity=leftWheelVelocity,force=1000)
    p.setJointMotorControl2(mobot.robotId,1,p.VELOCITY_CONTROL,targetVelocity=rightWheelVelocity,force=1000)

    mobot.get_observation()

