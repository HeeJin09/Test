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

######setting ###################
physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER,1)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS,0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetDebugVisualizerCamera(cameraDistance=1.3, cameraYaw=0.0, cameraPitch=-40, cameraTargetPosition=[0.0, 0, 0.5])

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

######load_urdf
planeID = p.loadURDF("plane.urdf")
boxID = p.loadURDF("C:/Users/whj03/Desktop/admittance/urdf/box2.urdf", [0, 0, 0.0],quat,useFixedBase=True)
Orientation = p.getQuaternionFromEuler([0, 0, 0])
arrowID = p.loadURDF("C:/Users/whj03/Desktop/admittance/urdf/abc.urdf", [0.48, -0.3, 1.2], quat)


p.setGravity(0, 0, 0)
p.setTimeStep(1/240.)
time_step = 1/240.



if __name__ == "__main__":

    joint_list = [0];
    joint_states = [0];
    count = 0;
    wrench_list = []
    force_scaling_factor = 0.05

    p.enableJointForceTorqueSensor(boxID, 0)
    x_lineId = p.addUserDebugLine(lineFromXYZ=[0, 0, 0], lineToXYZ=[0, 0, 0], lineColorRGB=[1, 0, 0], lineWidth=3, lifeTime=time_step)
    y_lineId = p.addUserDebugLine(lineFromXYZ=[0, 0, 0], lineToXYZ=[0, 0, 0], lineColorRGB=[0, 1, 0], lineWidth=3, lifeTime=time_step)
    z_lineId = p.addUserDebugLine(lineFromXYZ=[0, 0, 0], lineToXYZ=[0, 0, 0], lineColorRGB=[0, 1, 0], lineWidth=3,lifeTime=time_step)
    pos_x_lineId = p.addUserDebugLine(lineFromXYZ=[0, 0, 0], lineToXYZ=[0, 0, 0], lineColorRGB=[1, 0, 0], lineWidth=3, lifeTime=time_step)
    pos_y_lineId = p.addUserDebugLine(lineFromXYZ=[0, 0, 0], lineToXYZ=[0, 0, 0], lineColorRGB=[0, 1, 0], lineWidth=3, lifeTime=time_step)
    pos_z_lineId = p.addUserDebugLine(lineFromXYZ=[0, 0, 0], lineToXYZ=[0, 0, 0], lineColorRGB=[0, 1, 0], lineWidth=3, lifeTime=time_step)

    t = 0;
    r = [0, 0, 0]
    t_list = [];
    r_list = [];
    sim_list = [];

    wrench1 = np.array(p.getJointState(boxID, 0)[2])
    for i in range(400):
        '''
        External_Force = [0.0, -100.0, 0.0]
        External_pos = [0.3, 0.5, 1.5]
        if count == 50:
            p.applyExternalForce(boxID, 0,  External_Force, External_pos, p.WORLD_FRAME)
        '''

        wrench = np.array(p.getJointState(boxID, 0)[2])
        wrench2 = wrench - wrench1
        Wrench = np.array([wrench2[3], wrench2[4], wrench2[5], wrench2[0], wrench2[1], wrench2[2]]).T
        Wrench = Adjoint(RpToTrans(R, np.array([0, 0, 0]).T)) @ Wrench
        wrench_list.append(Wrench)

        f = np.array([Wrench[3], Wrench[4], Wrench[5]])
        norm_f = np.linalg.norm(f)
        u = f / norm_f;

        f_nx = np.dot(u, nx) * nx * norm_f * force_scaling_factor
        f_ny = np.dot(u, ny) * ny * norm_f * force_scaling_factor
        f_nz = np.dot(u, nz) * nz * norm_f * force_scaling_factor

        U = np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0], [n[0], n[1], n[2]]])
        position = [-0.5 + (count*0.005), -0.5 + (count*0.005), count%2 + 1.1]
        #p.resetBasePositionAndOrientation(arrowID, position, Orientation)


        try:
            r = np.linalg.inv(U.T @ U) @ U.T @ np.array([-Wrench[0] / norm_f, -Wrench[1] / norm_f, -Wrench[2] / norm_f, -d])
            print('contact point', r)
            a = p.getContactPoints(boxID, arrowID, 2, -1)
            sim_contact_point = np.array(a[0][5])
            print('simulation', sim_contact_point)
        except:
            r = [0, 0, 0]
            sim_contact_point = [0, 0, 0]
        r_list.append(r)
        sim_list.append(sim_contact_point)


        print(count)
        uf_nx = f_nx / np.linalg.norm(f_nx)
        uf_ny = f_ny / np.linalg.norm(f_ny)
        uf_nz = f_nz / np.linalg.norm(f_nz)
        p.addUserDebugLine([r[0] + 10 * uf_nx[0], r[1] + 10 * uf_nx[1], r[2] + 10 * uf_nx[2]], [r[0] - 10 * uf_nx[0], r[1] - 10 * uf_nx[1], r[2] - 10 * uf_nx[2]], [1, 0, 0], 1, time_step * 10, replaceItemUniqueId=x_lineId)
        p.addUserDebugLine([r[0] + 10 * uf_ny[0], r[1] + 10 * uf_ny[1], r[2] + 10 * uf_ny[2]], [r[0] - 10 * uf_ny[0], r[1] - 10 * uf_ny[1], r[2] - 10 * uf_ny[2]], [0, 1, 0], 1, time_step * 10, replaceItemUniqueId=y_lineId)
        p.addUserDebugLine([r[0] + 10 * uf_nz[0], r[1] + 10 * uf_nz[1], r[2] + 10 * uf_nz[2]], [r[0] - 10 * uf_nz[0], r[1] - 10 * uf_nz[1], r[2] - 10 * uf_nz[2]], [0, 0, 1], 1, time_step * 10, replaceItemUniqueId=z_lineId)

        p.addUserDebugLine([r[0], r[1], r[2]], [r[0] - f_nx[0], r[1] - f_nx[1], r[2] - f_nx[2]], [1, 0, 0], 5, time_step * 50, replaceItemUniqueId=pos_x_lineId)
        p.addUserDebugLine([r[0], r[1], r[2]], [r[0] - f_ny[0], r[1] - f_ny[1], r[2] - f_ny[2]], [0, 1, 0], 5, time_step * 50, replaceItemUniqueId=pos_y_lineId)
        p.addUserDebugLine([r[0], r[1], r[2]], [r[0] - f_nz[0], r[1] - f_nz[1], r[2] - f_nz[2]], [0, 0, 1], 5, time_step * 50, replaceItemUniqueId=pos_z_lineId)

        t = t + time_step
        t_list.append(t)
        count = count + 1;
        p.stepSimulation();
        time.sleep(time_step)


error_0 =  np.array([elem[0] for elem in sim_list]) - np.array([elem[0] for elem in r_list])
error_1 =  np.array([elem[1] for elem in sim_list]) - np.array([elem[1] for elem in r_list])
list_size = len(sim_list)
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(221)
plt.plot(np.array([elem[0] for elem in sim_list]), np.array([elem[1] for elem in sim_list]), 'bx', label='sim')
plt.plot(np.array([elem[0] for elem in r_list]), np.array([elem[1] for elem in r_list]), '.', label='estimation', color = "hotpink")
#plt.plot(np.array([elem[0] for elem in sim_list]), np.array([elem[0] for elem in sim_list]) - np.array([elem[0] for elem in r_list]), '0', label='error_x')
#plt.plot(np.array([elem[1] for elem in sim_list]), np.array([elem[1] for elem in sim_list]) - np.array([elem[1] for elem in r_list]), '0', label='error_y')

plt.xlabel('x')
plt.ylabel('y')

plt.title("estimation contact point")

ay = fig.add_subplot(222)
plt.plot(np.array([elem[0] for elem in sim_list]), np.array([elem[2] for elem in sim_list]), 'bx', label='sim')
plt.plot(np.array([elem[0] for elem in r_list]), np.array([elem[2] for elem in r_list]), '.', label='estimation', color = "hotpink")
#plt.plot(np.array([elem[0] for elem in sim_list]), np.array([elem[0] for elem in sim_list]) - np.array([elem[0] for elem in r_list]), '0', label='error_x')
#plt.plot(np.array([elem[1] for elem in sim_list]), np.array([elem[1] for elem in sim_list]) - np.array([elem[1] for elem in r_list]), '0', label='error_y')

plt.xlabel('x')
plt.ylabel('z')
plt.title("estimation contact point")

plt.legend()
plt.show()

