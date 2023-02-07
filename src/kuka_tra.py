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
box2ID = p.loadURDF("C:/Users/whj03/Desktop/admittance/urdf/box.urdf", [2, 0, 2.0])
robotID = p.loadURDF("C:/Users/whj03/Desktop/admittance/urdf/kr210l150.urdf", [-3.5, 0, 0.0])
p.setGravity(0, 0, 0)
p.setTimeStep(1/240.)
time_step = 1/240.
p.resetDebugVisualizerCamera(cameraDistance=6, cameraYaw=0.0, cameraPitch=-10, cameraTargetPosition=[0.0, 0, 0.5])


if __name__ == "__main__":
    joint_list = [1, 2, 3, 4, 5, 6];
    joint_states = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    count = 0;
    wrench_list = []
    estimation_pos_x = [0.0, 0.0]
    estimation_pos_y = [0.0, 0.0]
    estimation_pos_z = [0.0, 0.0]
    a = []
    t = 0
    t_list = []
    p.setJointMotorControlArray(robotID, joint_list, p.VELOCITY_CONTROL, forces=[0] * len(joint_list));
    for i in range(0,p.getNumJoints(robotID)):
        print(i,p.getJointInfo(robotID,i))

    box_end_pos = np.array(p.getLinkState(box2ID, 0, 1, 1)[0])
    box_end_orn = np.array(p.getLinkState(box2ID, 0, 1, 1)[1])
    kuka_end_pos = np.array(p.getLinkState(robotID, 7, 1, 1)[0])
    kuka_end_orn = np.array(p.getLinkState(robotID, 7, 1, 1)[1])
    end_pos = box_end_pos - kuka_end_pos
    end_orn = box_end_orn - kuka_end_orn

    for i in range(1000000):

        ExternalForce = [0.0, 100.0, 100.0]
        if count == 50:
            p.applyExternalForce(box2ID, -1, [0.0, 0.0, -1000.0], [2.3, -0.5, 1.5], p.WORLD_FRAME)

        box_end_pos = np.array(p.getLinkState(box2ID, 0, 1, 1)[0])
        kuka_end_pos = box_end_pos - end_pos
        box_end_orn = np.array(p.getLinkState(box2ID, 0, 1, 1)[1])
        kuka_end_orn1 = box_end_orn - end_orn
        kukaOrientation = p.getQuaternionFromEuler(kuka_end_orn1[0:3])
        inverse_joint_state = p.calculateInverseKinematics(robotID, 7, kuka_end_pos, kukaOrientation)

        print('box', box_end_orn)
        print('kuka',kuka_end_orn1)
        p.setJointMotorControlArray(robotID, joint_list, p.POSITION_CONTROL, inverse_joint_state)

        #print(box_end_pos[0] - kuka_end_pos[1])


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


