import pybullet as p
import numpy as np
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
#boxId = p.loadURDF("r2d2.urdf",cubeStartPos, cubeStartOrientation)
boxId = p.loadMJCF("mjcf/Ali/tendon_quadruped_ws_onfloor.xml")
number_of_joints = p.getNumJoints(2)
maxForce = 0
mode = p.VELOCITY_CONTROL
p.setJointMotorControlArray(2, np.array(range(number_of_joints)),controlMode=mode, forces=np.zeros(number_of_joints))

for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()

#import pdb; pdb.set_trace()
