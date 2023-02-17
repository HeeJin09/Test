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

########contact point setting ########
eul_x = 0.0
eul_y = 0.0
eul_z = 0.0
quat = p.getQuaternionFromEuler([eul_x, eul_y, eul_z])
d = -1.0
R = p.getMatrixFromQuaternion(quat)
R = np.reshape(R, [3, 3])
n = R@np.array([0, 0, 1]).T
nx = R@np.array([1, 0, 0]).T
ny = R@np.array([0, 1, 0]).T
nz = R@np.array([0, 0, 1]).T
force_scaling_factor = 0.05

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeID = p.loadURDF("plane.urdf")
boxID = p.loadURDF("C:/Users/whj03/Desktop/admittance/urdf/box2.urdf", [0, 0, 0.0], quat,useFixedBase=True)
box2ID = p.loadURDF("C:/Users/whj03/Desktop/admittance/urdf/box.urdf", [2, 0, 0.75], quat)
arrowID = p.loadURDF("C:/Users/whj03/Desktop/admittance/urdf/abc.urdf", [0.75, -0.75, 1.2], quat)

p.setGravity(0, 0, 0)
p.setTimeStep(1/240.)
time_step = 1/240.
p.resetDebugVisualizerCamera(cameraDistance=2.2, cameraYaw=0.0, cameraPitch=0, cameraTargetPosition=[1, 0, 0.5])

p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER,1)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS,0)
if __name__ == "__main__":
    joint_list = [0];
    joint_states = [0];count = 0;
    wrench_list = []
    wrench1 = np.array(p.getJointState(boxID, 0)[2])

    p.enableJointForceTorqueSensor(boxID, 0)
    a = []
    t = 0
    t_list = []
    r = [2, 0, 0.75]
    for i in range(1000):
        wrench = np.array(p.getJointState(boxID, 0)[2])
        wrench2 = wrench - wrench1
        Wrench = np.array([wrench2[3], wrench2[4], wrench2[5], wrench2[0], wrench2[1], wrench2[2]]).T
        Wrench = Adjoint(RpToTrans(R, np.array([0, 0, 0]).T)) @ Wrench
        wrench_list.append(Wrench)

        f = np.array([Wrench[3], Wrench[4], Wrench[5]])
        norm_f = np.linalg.norm(f)
        u = f / norm_f;
        U = np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0], [n[0], n[1], n[2]]])


        try:
            r = np.linalg.inv(U.T @ U) @ U.T @ np.array([-Wrench[0] / norm_f, -Wrench[1] / norm_f, -Wrench[2] / norm_f, -d])
        except:
            r = [0, 0, 0.0]
        r = np.where(np.isnan(r), 0 , r)
        p.applyExternalForce(box2ID, -1, f * -0.005, [2 + r[0], 0 + r[1], 0.75 + (1-r[2])], p.WORLD_FRAME)
        print('contact point', r)



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
