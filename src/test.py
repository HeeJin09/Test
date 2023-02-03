import os
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

TFID1 = p.loadURDF("C:/Users/whj03/Desktop/admittance/urdf/TFtest.urdf", [0, 0, 0.15])
boxID = p.loadURDF("C:/Users/whj03/Desktop/admittance/urdf/box.urdf", [0, 0, 0.8])
p.setGravity(0, 0, -9.8)
p.setTimeStep(1/240.)
time_step = 1/240.
p.resetDebugVisualizerCamera(cameraDistance=2, cameraYaw=0.0, cameraPitch=10, cameraTargetPosition=[0.0, 0, 0.5])
arrowID = p.loadURDF("C:/Users/whj03/Desktop/admittance/urdf/abc.urdf", [10, 0, 0])
cid = p.createConstraint(TFID1, 0, boxID, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0.8], [0, 0, 0])
p.changeConstraint(cid, maxForce=500)

if __name__ == "__main__":
	joint_list = [0];
	joint_states = [0];
	count = 0;
	wrench_list = []

	p.enableJointForceTorqueSensor(TFID1, 0)

	t = 0;
	t_list = [];
	for i in range(30000):

		wrench = np.array(p.getJointState(TFID1, 0)[2])
		if count == 50:
			p.applyExternalForce(boxID, 0,  [0.0, 100.0, 100.0], [0.0, 0.0, 0.0], p.WORLD_FRAME)


		print(wrench)
		count = count + 1;
		p.stepSimulation();
		time.sleep(time_step)

'''
list_size = len(wrench_list)
fig = plt.figure(figsize=(10, 7))
#X =np.arange(t_list)
ax = fig.add_subplot(221)
plt.plot(t_list, np.array([elem[3] for elem in wrench_list]), label='fx')
plt.plot(t_list, np.array([elem[4] for elem in wrench_list]), label='fy')
plt.plot(t_list, np.array([elem[5] for elem in wrench_list]), label='fz')
plt.ylim([-5, 30])
plt.xlabel('time[s]')
plt.ylabel('[N]')

plt.title("measured force/torque")


list_size_1 = len(pos_list)
ay = fig.add_subplot(222)
plt.plot(t_list, np.array([pos_list[3] for pos_list in pos_list]), label='pos_x')
plt.plot(t_list, np.array([pos_list[4] for pos_list in pos_list]), label='pos_y')
plt.plot(t_list, np.array([pos_list[5] for pos_list in pos_list]), label='pos_z')
plt.xlabel('time[s]')
plt.ylabel('[m]')
plt.ylim([-0.1, 0.15])
plt.title("measured position")

plt.legend()
plt.show()
'''
