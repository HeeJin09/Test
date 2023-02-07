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
box2ID = p.loadURDF("C:/Users/whj03/Desktop/admittance/urdf/box.urdf", [2, 0, 1.0])
p.setGravity(0, 0, 0)
p.setTimeStep(1/240.)
time_step = 1/240.
p.resetDebugVisualizerCamera(cameraDistance=4, cameraYaw=0.0, cameraPitch=0, cameraTargetPosition=[0.8, 0, 0.5])


if __name__ == "__main__":
    joint_list = [0];
    joint_states = [0];count = 0;
    wrench_list = []
    estimation_pos_x = [0.0, 0.0]
    estimation_pos_y = [0.0, 0.0]
    estimation_pos_z = [0.0, 0.0]

    p.enableJointForceTorqueSensor(boxID, 0)
    a = []
    t = 0
    t_list = []
    for i in range(1000):

        ExternalForce = [0.0, 100.0, 100.0]
        if count == 50:
            p.applyExternalForce(boxID, 1,  [0.0, 0.0, -1000], [0.3, -0.5, 1.5], p.WORLD_FRAME)
            p.applyExternalForce(box2ID, -1, [0.0, 0.0, -1000.0], [2.3, -0.5, 1.5], p.WORLD_FRAME)

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
        t = t + time_step
        t_list.append(t)
        count = count + 1;
        p.stepSimulation();
        time.sleep(time_step)


list_size = len(wrench_list)
fig = plt.figure(figsize=(10, 7))
#X =np.arange(t_list)
ax = fig.add_subplot(221)
plt.plot(t_list, np.array([elem[3] for elem in wrench_list]), label='fx')
plt.plot(t_list, np.array([elem[4] for elem in wrench_list]), label='fy')
plt.plot(t_list, np.array([elem[5] for elem in wrench_list]), label='fz')
plt.ylim([-500, 300])
plt.xlabel('time[s]')
plt.ylabel('[N]')

plt.title("measured force/torque")

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
