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
cubeStartPos = [0, 0, 0]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 3.14])
robotID = p.loadURDF("C:/Users/whj03/Desktop/admittance/urdf/indy7.urdf", cubeStartPos, cubeStartOrientation)
p.setGravity(0, 0, 0)
p.setTimeStep(1/200)
time_step = 1/200
p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0.1, cameraPitch=-20, cameraTargetPosition=[0.3, 0, 0.5])
arrowID = p.loadURDF("C:/Users/whj03/Desktop/admittance/urdf/abc.urdf", [10, 0, 0], cubeStartOrientation)
for i in range(0,p.getNumJoints(robotID)):
	print(i,p.getJointInfo(robotID,i))

def getJointPosition():
	JointStates = p.getJointStates(robotID, joint_list);
	JointPositions = [JointStates[0][0],JointStates[1][0],JointStates[2][0],JointStates[3][0],JointStates[4][0],JointStates[5][0]]
	return JointPositions

def getJointVelocities():
	JointStates = p.getJointStates(robotID, joint_list);
	JointVelocities = [JointStates[0][1],JointStates[1][1],JointStates[2][1],JointStates[3][1],JointStates[4][1],JointStates[5][1]]
	return JointVelocities



def w_p_to_slit():
	Slist =  [
		[0.0, 4.8965888601467475e-12, 4.8965888601467475e-12, 2.3976582465313225e-23, 9.793177720293495e-12,7.1930093584727e-23],
		[0.0, -1.0, -1.0, -4.8965888601227706e-12, -1.0, -9.793177720245541e-12],
		[1.0, 2.3976582465313225e-23, 2.3976582465313225e-23, 1.0, -4.896588860098794e-12, 1.0],
		[-0.0, 0.2995, 0.7495, -0.0034999999950226205, 1.0995000000000172, -0.18649999998881622],
		[-0.0, 1.4665283636139508e-12, 3.6699933506799874e-12, 4.027444337495072e-12, 1.0767598903440245e-11,3.6161308733035264e-12],
		[-0.0, 5.337281857559955e-13, 2.737193172822032e-12, 1.980465711625216e-23, 4.6199315895504464e-12, 4.8828374754675285e-23]]
	Blist = [
		[2.3976928654100554e-23, -4.8965888601467475e-12, -4.8965888601467475e-12, 3.4618902214045736e-28, -1.1740516174705233e-34, 6.9237804428385345e-28],
		[9.793177720245541e-12, -1.0, -1.0, 4.8965888601227706e-12, -1.0, 0.0],
		[1.0, 9.793177720269518e-12, 9.793177720269518e-12, 1.0, 4.8965888601467475e-12, 1.0],
		[0.18650000000181666, -1.0279999999991043, -0.5779999999991041, 0.18300000000029382, -0.2280000000000002, 3.5413412302101557e-23],
		[-5.4425585180605675e-12, 5.03369334818734e-12, 2.830228361142882e-12, -1.380838058561384e-12, -4.0283652976134964e-24, -4.038963052882063e-28],
		[4.882824562617003e-23, -3.995616509902955e-12, -1.7921515228261296e-12, 6.761330479271026e-24, -8.226269285046539e-13, -1.2912850525166677e-28]]
	Mlist = [
		[[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0775], [0.0, 0.0, 0.0, 1.0]],
		[[4.8965888601467475e-12, 1.0, 4.8965888601467475e-12, 0.0],[0.0, 4.8965888601467475e-12, -1.0, -0.109],[-1.0, 4.8965888601467475e-12, 2.3976582465313225e-23, 0.22199999999999998],[0.0, 0.0, 0.0, 1.0]],
		[[1.0, 0.0, 0.0, -0.45000000000000007], [0.0, 1.0, 1.1740346660404259e-34, 0.0],[0.0, 1.1740346660404259e-34, 1.0, -0.0305], [0.0, 0.0, 0.0, 1.0]],
		[[4.8965888601467475e-12, -4.8965888601467475e-12, -1.0, -0.2670000000000001],[1.0, 2.397658246519582e-23, 4.8965888601467475e-12, 8.077935669463161e-28], [0.0, -1.0, 4.8965888601467475e-12, -0.075], [0.0, 0.0, 0.0, 1.0]],
		[[4.8965888601467475e-12, 1.0, 4.8965888601467475e-12, 0.0],[0.0, 4.8965888601467475e-12, -1.0, -0.11399999999999999],[-1.0, 4.8965888601467475e-12, 2.3976928654100554e-23, 0.08299999999999996], [0.0, 0.0, 0.0, 1.0]],
		[[4.896588860146747e-12, -4.896588860146747e-12, -1.0, -0.16800000000000015],[1.0, 2.3976236276525892e-23, 4.896588860146748e-12, 6.058451752097371e-28], [-1.1740516174705233e-34, -1.0, 4.8965888601467475e-12, 0.06900000000000003],[0.0, 0.0, 0.0, 1.0]],
		[[1.0, 2.3481032349525446e-34, 6.9237804428385345e-28, -8.077935669463161e-28],[2.3481032349525446e-34, 1.0, 0.0, 0.0], [6.9237804428385345e-28, 0.0, 1.0, 0.06000000000000005],[0.0, 0.0, 0.0, 1.0]]]
	Glist = [
			[[0.15418559, -2.35e-06, 1.739e-05, 0.0, 0.0, 0.0], [-2.35e-06, 0.12937017, -0.04854267, 0.0, 0.0, 0.0],
			 [1.739e-05, -0.04854267, 0.05964415, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 11.8030102, 0.0, 0.0],
			 [0.0, 0.0, 0.0, 0.0, 11.8030102, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 11.8030102]],
			[[0.2935698, -4e-07, 1.441e-05, 0.0, 0.0, 0.0], [-4e-07, 0.28094142, 0.03727972, 0.0, 0.0, 0.0],
			 [1.441e-05, 0.03727972, 0.03620609, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 7.99292141, 0.0, 0.0],
			 [0.0, 0.0, 0.0, 0.0, 7.99292141, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 7.99292141]],
			[[0.03424593, 1.49e-06, 7.24e-06, 0.0, 0.0, 0.0], [1.49e-06, 0.03406024, 0.00186009, 0.0, 0.0, 0.0],
			 [7.24e-06, 0.00186009, 0.00450477, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 2.99134127, 0.0, 0.0],
			 [0.0, 0.0, 0.0, 0.0, 2.99134127, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 2.99134127]],
			[[0.00670405, 3.75e-06, 1.5e-06, 0.0, 0.0, 0.0], [3.75e-06, 0.00279246, -0.00127967, 0.0, 0.0, 0.0],
			 [1.5e-06, -0.00127967, 0.00619341, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 2.12317035, 0.0, 0.0],
			 [0.0, 0.0, 0.0, 0.0, 2.12317035, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 2.12317035]],
			[[0.00994891, 1.4e-07, 3.21e-06, 0.0, 0.0, 0.0], [1.4e-07, 0.00978189, -0.00093546, 0.0, 0.0, 0.0],
			 [3.21e-06, -0.00093546, 0.00271492, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 2.28865091, 0.0, 0.0],
			 [0.0, 0.0, 0.0, 0.0, 2.28865091, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 2.28865091]],
			[[0.00043534, 1.3e-07, -2e-08, 0.0, 0.0, 0.0], [1.3e-07, 0.00044549, 5.1e-07, 0.0, 0.0, 0.0],
			 [-2e-08, 5.1e-07, 0.00059634, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.40083918, 0.0, 0.0],
			 [0.0, 0.0, 0.0, 0.0, 0.40083918, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.40083918]]]
	M = [[1.0, -9.793177720293495e-12, 7.1930093584727e-23, -3.6161308732080396e-12],
				  [9.793177720293495e-12, 1.0, -9.793177720245541e-12, -0.18650000000181666],
				  [2.3976928654100554e-23, 9.793177720245541e-12, 1.0, 1.3274999999991042], [0.0, 0.0, 0.0, 1.0]]

	return Slist, Blist, Mlist, Glist, M

if __name__ == "__main__":
	joint_list = [1, 2, 3, 4, 5, 6];
	joint_states = [0, 0.3925, 0.785, 0, 0.3925, 0];
	count = 0;
	wrench_list = []
	wrench_list1 = []
	Teef = np.eye(4);

	for i in range(len(joint_list)):
		p.resetJointState(robotID, joint_list[i], joint_states[i]);

	p.enableJointForceTorqueSensor(robotID,6, 1)
	wrench_list = []
	wrench_list1 = []
	pos_list = []

	pos1 = np.array(p.getLinkState(robotID, 7 , 1, 1)[0])
	pos2 = [pos1[0], pos1[1], pos1[2]]
	orn1 = np.array(p.getLinkState(robotID, 7, 1, 1)[1])
	p.setJointMotorControlArray(robotID, joint_list, p.VELOCITY_CONTROL, forces=[0] * len(joint_list));
	p.enableJointForceTorqueSensor(robotID, 7, 1)

	Teef[0:3, 3] = np.array(pos1).T
	Teef[0:3, 0:3] = np.reshape(p.getMatrixFromQuaternion(orn1), (3, 3))
	prev_end_pos =  [0, 0, 0, 0, 0, 0];
	end_vel = np.array([0, 0, 0, 0, 0, 0]);
	end_acc = np.array([0, 0, 0, 0, 0, 0]);
	M = 0;
	Wn = 10;
	zeta = 2**(1/2) ;

	B1 = 30;
	K1 = 40;
	B = np.array([[B1, 0, 0, 0, 0, 0],[0, B1, 0, 0, 0, 0],[0, 0, B1, 0, 0, 0],[0, 0, 0, B1, 0, 0],[0, 0, 0, 0, B1, 0],[0, 0, 0, 0, 0, B1]]);
	K = np.array([[K1, 0, 0, 0, 0, 0],[0, K1, 0, 0, 0, 0],[0, 0, K1, 0, 0, 0],[0, 0, 0, K1, 0, 0],[0, 0, 0, 0, K1, 0],[0, 0, 0, 0, 0, K1]]);
	prev_theta_dot = 0;
	prev_theta = joint_states;
	t = 0;
	t_list = [];
	print(B);
	print(K);
	for i in range(300):

		# **** Calculate Position and Velocity of Joint ******
		Joint_pos = getJointPosition();
		Joint_velocity = getJointVelocities();
		Joint_Acceleraions = [0, 0, 0, 0, 0, 0]

		Slist, Blist, Mlist, Glist, M_1 = w_p_to_slit();
		T_ = FKinBody(M, Blist, Joint_pos)
		Jb = JacobianBody(Blist, Joint_pos)
		Js = JacobianSpace(Slist, Joint_pos)
		#Jb_pseudo = lin.pinv(Jb) + np.identity(6)*0.000001
		Jb_pseudo = lin.pinv(Jb)
		# ****** gravity compensation ******
		grav_comp_torque = p.calculateInverseDynamics(robotID, Joint_pos,Joint_velocity, Joint_Acceleraions)
		applied_torque = grav_comp_torque
		#p. setJointMotorControlArray(robotID, joint_list, p.TORQUE_CONTROL, forces=applied_torque[0:6]);


		if count < 2:
			wrench1 = np.array(p.getJointState(robotID, 7)[2])
			pos1 = np.array(p.getLinkState(robotID, 7, 1, 1)[0])
			orn1 = np.array(p.getLinkState(robotID, 7, 1, 1)[1])
			end_pos = np.array([orn1[0], orn1[1], orn1[2], pos1[0], pos1[1], pos1[2]])

		wrench = np.array(p.getJointState(robotID, 7)[2])
		wrench2 = wrench - wrench1
		wrench3 = [wrench2[3], wrench2[4], wrench2[5], wrench2[0], wrench2[1], wrench2[2]]
		wrench_list.append(wrench3)
		Orientation = p.getQuaternionFromEuler([0, -1.57, 0])

		pos2 = np.array(p.getLinkState(robotID, 7, 1, 1)[0])
		orn2 = np.array(p.getLinkState(robotID, 7, 1, 1)[1])
		end_pos1 = np.array([orn2[0], orn2[1], orn2[2], pos2[0], pos2[1], pos2[2]])
		end_pos2 = end_pos - end_pos1 ;
		pos_list.append(end_pos2)
		position = [pos2[0] + 100, pos2[1], pos2[2]+ 100]
		if count == 50:
			p.applyExternalForce(robotID, 7,  [10.0, 0.0, 0.0], [0.0, 0.0, 0.0], p.WORLD_FRAME)

		if count >= 50 and count < 80:
			position = [pos2[0]+ 0.3, pos2[1], pos2[2]]
		elif count>=80:
			position = [pos2[0], pos2[1], pos2[2]+ 100]
		p.resetBasePositionAndOrientation(arrowID, position, Orientation)

		print(count)


		#for i in range(len(end_pos)):
			#end_vel[i] = (1/B)*(wrench3[i] - (K * end_pos2[i]));

		end_vel = lin.inv(B) @ (wrench3 - (K @ end_pos2));
		prev_end_pos = np.array(end_pos2);
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