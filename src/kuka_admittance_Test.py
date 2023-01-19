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
p.setGravity(0, 0, 0)
p.setTimeStep(1/200)
time_step = 1/200

for i in range(0,p.getNumJoints(robotID)):
	print(i,p.getJointInfo(robotID,i))

if __name__ == "__main__":

		p.stepSimulation();
		time.sleep(time_step)
