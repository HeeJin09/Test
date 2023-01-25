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
	p_.append([-9.8483e-05 + 0.35277 - 0.00262, -0.1475 - 0.037476 + 0.00097586, 1.2499 + 0.4192 + 0.33099 + 0.4192])
	p_.append([0.95795 - 9.8483e-05 + 0.35277 - 0.00262, 0.184 - 0.1475 - 0.037476 + 0.00097586,
			   -0.055059 + 1.2499 + 0.4192 + 0.33099 + 0.4192])
	p_.append([0.542 + 0.95795 - 9.8483e-05 + 0.35277 - 0.00262, 0 + 0.184 - 0.1475 - 0.037476 + 0.00097586,
			   -0.055059 + 1.2499 + 0.4192 + 0.33099 + 0.4192])
	p_.append([0.1925 + 0.542 + 0.95795 - 9.8483e-05 + 0.35277 - 0.00262, 0 + 0.184 - 0.1475 - 0.037476 + 0.00097586,
			   -0.055059 + 1.2499 + 0.4192 + 0.33099 + 0.4192])
	p_temp = p_[-1]

	M = np.eye(4);
	M[0, 3] = 0.1925 + 0.542 + 0.95795 - 9.8483e-05 + 0.35277 - 0.00262
	M[1, 3] = 0 + 0.184 - 0.1475 - 0.037476 + 0.00097586
	M[2, 3] = -0.055059 + 1.2499 + 0.4192 + 0.33099 + 0.4192
	Slist = w_p_to_slit(w, p_);
	Blist = Adjoint(TransInv(M)) @ Slist;

	joint_list = [1, 2, 3, 4, 5, 6, 7];
	joint_states = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
	count = 0;
	pos_list = []
	wrench_list = []
	wrench_list1 = []
	Teef = np.eye(4);
	t = 0;
	t_list = [];

	pos1 = np.array(p.getLinkState(robotID, 8, 1, 1)[0])
	pos2 = [pos1[0], pos1[1], pos1[2]]
	orn1 = np.array(p.getLinkState(robotID, 8, 1, 1)[1])
	Teef[0:3, 3] = np.array(pos1).T
	Teef[0:3, 0:3] = np.reshape(p.getMatrixFromQuaternion(orn1), (3, 3))
	prev_end_pos = [0, 0, 0, 0, 0, 0];
	end_vel = np.array([0, 0, 0, 0, 0, 0]);
	end_acc = np.array([0, 0, 0, 0, 0, 0]);

	B1 = 0;
	K1 = 0;
	B = np.array(
		[[B1, 0, 0, 0, 0, 0], [0, B1, 0, 0, 0, 0], [0, 0, B1, 0, 0, 0], [0, 0, 0, B1, 0, 0], [0, 0, 0, 0, B1, 0],
		 [0, 0, 0, 0, 0, B1]]);
	K = np.array(
		[[K1, 0, 0, 0, 0, 0], [0, K1, 0, 0, 0, 0], [0, 0, K1, 0, 0, 0], [0, 0, 0, K1, 0, 0], [0, 0, 0, 0, K1, 0],
		 [0, 0, 0, 0, 0, K1]]);

	prev_theta_dot = 0;
	prev_theta = joint_states;

	p.setJointMotorControlArray(robotID, joint_list, p.VELOCITY_CONTROL, forces=[0] * len(joint_list));
	p.enableJointForceTorqueSensor(robotID, 8, 1)

	for i in range(len(joint_list)):
		p.resetJointState(robotID, joint_list[i], joint_states[i]);

	for i in range(10000):

		Joint_pos = getJointPosition(robotID, joint_list);
		Joint_velocity = getJointVelocities(robotID, joint_list);
		Joint_Acceleraions = [0, 0, 0, 0, 0, 0, 0]


		T_ = FKinBody(M, Blist, Joint_pos)
		Jb = JacobianBody(Blist, Joint_pos)
		Js = JacobianSpace(Slist, Joint_pos)
		#Jb_pseudo = lin.pinv(Jb) + np.identity(6) * 0.00001
		Jb_pseudo = lin.pinv(Jb)

		grav_comp_torque = p.calculateInverseDynamics(robotID, Joint_pos, Joint_velocity, Joint_Acceleraions)
		applied_torque = grav_comp_torque

		wrench = np.array(p.getJointState(robotID, 7)[2])
		wrench1 = [wrench[3], wrench[4], wrench[5], wrench[0], wrench[1], wrench[2]]
		wrench_list.append(wrench1)

		if count == 50:
			p.applyExternalForce(robotID, 8,  [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], p.WORLD_FRAME)

		pos2 = np.array(p.getLinkState(robotID, 8, 1, 1)[0])
		orn2 = np.array(p.getLinkState(robotID, 8, 1, 1)[1])
		end_pos1 = np.array([orn2[0], orn2[1], orn2[2], pos2[0], pos2[1], pos2[2]])
		pos_list.append(end_pos1)

		end_vel = lin.inv(B) @ (wrench1 - (K @ end_pos1));
		prev_end_pos = np.array(end_pos1);

		theta_dot = Jb_pseudo @ end_vel
		theta = (theta_dot * time_step) + prev_theta;
		prev_theta = theta;

		p.setJointMotorControlArray(robotID, joint_list, p.POSITION_CONTROL, targetPositions=theta);
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
plt.ylim([-50, 1200])
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
plt.ylim([-0.2, 0.5])
plt.title("measured position")

plt.legend()
plt.show()