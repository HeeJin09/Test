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
boxID = p.loadURDF("C:/Users/whj03/Desktop/admittance/urdf/box2.urdf", [0, 0, 0.0])
p.setGravity(0, 0, 0)
p.setTimeStep(1/240.)
time_step = 1/240.
p.resetDebugVisualizerCamera(cameraDistance=2, cameraYaw=0.0, cameraPitch=0, cameraTargetPosition=[0.0, 0, 0.5])
Orientation = p.getQuaternionFromEuler([0, -3.14, 0])
arrowID = p.loadURDF("C:/Users/whj03/Desktop/admittance/urdf/abc.urdf", [0.48, -0.3, 15],Orientation)
if __name__ == "__main__":
	joint_list = [0];
	joint_states = [0];
	count = 0;
	wrench_list = []
	estimation_pos_x = [0.0, 0.0]
	estimation_pos_y = [0.0, 0.0]
	estimation_pos_z = [0.0, 0.0]
	p.enableJointForceTorqueSensor(boxID, 0)
	a = []
	t = 0;
	t_list = [];
	for i in range(55):

		External_Force = [-100.0, -100.0, 0.0]
		External_pos = [0.3, 0.5, 1.5]
		if count == 50:
			p.applyExternalForce(boxID, 2,  External_Force, External_pos, p.WORLD_FRAME)

		'''
		if count == 50:
			position =  [0.48, -0.3, 1.52]
		else:
			position = [0.48, -0.3, 1.509 + 1.6]
		p.resetBasePositionAndOrientation(arrowID, position, Orientation)
		'''

		wrench = np.array(p.getJointState(boxID, 0)[2])
		wrench2 = wrench
		wrench_list.append(wrench2)

		estimation_pos_x = [-(wrench2[5]/wrench2[0]), (wrench2[4]/wrench2[0])]
		estimation_pos_y = [(wrench2[5] / wrench2[1]), -(wrench2[3] / wrench2[1])]
		estimation_pos_z = [-(wrench2[4] / wrench2[2]), (wrench2[3] / wrench2[2])]
		a = p.getContactPoints(boxID, 1)

		print(count)

		print('moment ', np.array([wrench2[3], wrench2[4], wrench2[5]]))
		print('FORCE ', np.array([wrench2[0], wrench2[1], wrench2[2]]))

		print('(y,z)', estimation_pos_x)
		print('(x,z)', estimation_pos_y)
		print('(x,y)', estimation_pos_z)
		print('contact point ',  a)
		if count == 51:
			y1 = estimation_pos_x[0]
			z1 = estimation_pos_x[1]
			x2 = estimation_pos_y[0]
			z2 = estimation_pos_y[1]

		t = t + time_step
		t_list.append(t)
		count = count + 1;
		p.stepSimulation();
		time.sleep(time_step)

	ex_x = External_pos[0]
	ex_y = External_pos[1]
	ex_z = External_pos[2]

	#plt.plot(x, z, 'bx', label='sim')
	#plt.plot(x, ex_z, '.', label='ex', color = "hotpink")
	#plt.ylim([-2,2])
	#plt.xlim([-0.6, 0.6])
	#plt.xlabel('x')
	#plt.ylabel('z')
	#plt.show()

	#plt.title("measured force/torque")

'''
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
