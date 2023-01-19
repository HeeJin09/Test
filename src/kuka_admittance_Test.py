import pybullet as p
import time
import pybullet_data
import numpy as np
import math
import threading
import copy
import matplotlib.pyplot as plt
from modern_robotics import *
import numpy.linalg as lin

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeID = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 0]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 3.14])
robotID = p.loadURDF("C:/Users/whj03/Desktop/admittance/urdf/kr210l150_link.urdf", cubeStartPos, cubeStartOrientation)
p.setGravity(0, 0, -9.8)
p.setTimeStep(1/200)
time_step = 1/200

for i in range(0, p.getNumJoints(robotID)):
	print(i, p.getJointInfo(robotID, i))

def getJointPosition(ID,join_list):
	JointStates = p.getJointStates(ID, join_list);
	JointPositions = [JointStates[0][0], JointStates[1][0], JointStates[2][0], JointStates[3][0], JointStates[4][0], JointStates[5][0]]
	return JointPositions

def getJointVelocities(ID,join_list):
	JointStates = p.getJointStates(ID, join_list);
	JointVelocities = [JointStates[0][1], JointStates[1][1], JointStates[2][1], JointStates[3][1], JointStates[4][1], JointStates[5][1]]
	return JointVelocities
if __name__ == "__main__":
	for i in range(10000):
		p.stepSimulation();
		time.sleep(time_step)
