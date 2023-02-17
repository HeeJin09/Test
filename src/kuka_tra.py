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
from scipy.spatial.transform import Rotation as R

physicsClient = p.connect(p.GUI)

p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER,1)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS,0)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeID = p.loadURDF("plane.urdf")
Orientation = p.getQuaternionFromEuler([0, 1.57, 0])
box2ID = p.loadURDF("C:/Users/whj03/Desktop/admittance/urdf/box.urdf", [2, 0, 2.0], Orientation)
robotID = p.loadURDF("C:/Users/whj03/Desktop/admittance/urdf/kr210l150.urdf", [-3.5, 0, 0.0])
p.setGravity(0, 0, 0)
p.setTimeStep(1/240.)
time_step = 1/240.
p.resetDebugVisualizerCamera(cameraDistance=6, cameraYaw=0.0, cameraPitch=-10, cameraTargetPosition=[0.0, 0, 0.5])

def w_p_to_slit(w, p):
    Slist =[];
    for i in range(0,len(w)):
        v = -np.cross(np.array(w[i]), np.array(p[i]))
        wi = np.array(w[i])
        S = [wi[0], wi[1], wi[2], v[0], v[1], v[2]]
        Slist.append(S)
    return np.array(Slist).T
def getJointPosition(ID,join_list):
    JointPositions=[];
    JointStates = p.getJointStates(ID, join_list);
    for i in range(len(join_list)):
        JointPositions.append(JointStates[i][0])
    return JointPositions

def getJointVelocities(ID,join_list):
    JointVelocities = [];
    JointStates = p.getJointStates(ID, join_list);
    for i in range(len(join_list)):
        JointVelocities.append(JointStates[i][1])
    return JointVelocities

if __name__ == "__main__":
    w = []
    w.append([1, 0, 0])
    w.append([0, 0, 1])
    w.append([0, 1, 0])
    w.append([0, 1, 0])
    w.append([1, 0, 0])
    w.append([0, 1, 0])
    w.append([1, 0, 0])

    p_ = []
    p_.append([0.0, 0.0, 0.38])
    p_.append([-0.00262, 0.00097586, 0.33099 + 0.38])
    p_.append([0.35277 - 0.00262, -0.037476 + 0.00097586, 0.4192 + 0.33099 + 0.38])
    p_.append([-9.8483e-05 + 0.35277 - 0.00262, -0.1475 - 0.037476 + 0.00097586, 1.2499 + 0.4192 + 0.33099 + 0.38])
    p_.append([0.95795 - 9.8483e-05 + 0.35277 - 0.00262, 0.184 - 0.1475 - 0.037476 + 0.00097586,
               -0.055059 + 1.2499 + 0.4192 + 0.33099 + 0.38])
    p_.append([0.542 + 0.95795 - 9.8483e-05 + 0.35277 - 0.00262, 0 + 0.184 - 0.1475 - 0.037476 + 0.00097586,
               -0.055059 + 1.2499 + 0.4192 + 0.33099 + 0.38])
    p_.append([0.1925 + 0.542 + 0.95795 - 9.8483e-05 + 0.35277 - 0.00262, 0 + 0.184 - 0.1475 - 0.037476 + 0.00097586,
               -0.055059 + 1.2499 + 0.4192 + 0.33099 + 0.38])
    p_temp = p_[-1]

    M = np.eye(4);
    M[0, 3] = 0.1925 + 0.542 + 0.95795 - 9.8483e-05 + 0.35277 - 0.00262
    M[1, 3] = 0 + 0.184 - 0.1475 - 0.037476 + 0.00097586
    M[2, 3] = -0.055059 + 1.2499 + 0.4192 + 0.33099 + 0.38
    Slist = w_p_to_slit(w, p_);
    Blist = Adjoint(TransInv(M)) @ Slist;

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


    box_R = np.reshape(p.getMatrixFromQuaternion(box_end_orn),(3,3))
    box_p = box_end_pos
    box_T =RpToTrans(box_R,box_p)


    kuka_end_pos = np.array(p.getLinkState(robotID, 7, 1, 1)[0])
    kuka_end_orn = np.array(p.getLinkState(robotID, 7, 1, 1)[1])

    eef_R = np.reshape(p.getMatrixFromQuaternion(kuka_end_orn),(3,3))
    eef_p = kuka_end_pos
    eef_T =RpToTrans(eef_R,eef_p)

    Tbe=TransInv(box_T) @ eef_T

    end_pos = box_end_pos - kuka_end_pos
    end_orn = box_end_orn - kuka_end_orn

    for i in range(1000000):

        ExternalForce = [0.0, 100.0, 100.0]
        if count == 50:
            p.applyExternalForce(box2ID, -1, [0.0, 0.0, -1000.0], [2.3, -0.5, 1.5], p.WORLD_FRAME)

        Joint_pos = getJointPosition(robotID, joint_list);
        Joint_velocity = getJointVelocities(robotID, joint_list);
        Joint_Acceleraions = [0, 0, 0, 0, 0, 0, 0]

        T_ = FKinBody(M, Blist, Joint_pos)
        box_end_pos = np.array(p.getLinkState(box2ID, 0, 1, 1)[0])
        box_end_orn = np.array(p.getLinkState(box2ID, 0, 1, 1)[1])

        box_R = np.reshape(p.getMatrixFromQuaternion(box_end_orn), (3, 3))
        box_p = box_end_pos
        box_T = RpToTrans(box_R, box_p)

        theta = np.pi/2;

        T = np.array([[np.cos(theta), 0, np.sin(theta), 0.25], [0, 1, 0, 0], [-np.sin(theta), 1, np.cos(theta), 0], [0, 0, 0, 1]])
        EEF_T = box_T@Tbe
        EEF_R,EEF_p =TransToRp(EEF_T);
        EEF_r= R.from_matrix(EEF_R)
        EEF_quat =EEF_r.as_quat()

        inverse_joint_state = p.calculateInverseKinematics(robotID, 7, [EEF_T[0, 3], EEF_T[1, 3], EEF_T[2, 3]], [EEF_quat[0], EEF_quat[1], EEF_quat[2], EEF_quat[3]])
        p.setJointMotorControlArray(robotID, joint_list, p.POSITION_CONTROL, inverse_joint_state)



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


