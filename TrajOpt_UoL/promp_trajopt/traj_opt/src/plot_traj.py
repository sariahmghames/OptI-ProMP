#!/usr/bin/env python


import os
import sys
import rospy
import numpy as np
from operator import itemgetter
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import tf.transformations as tf_tran
import matplotlib.pyplot as plt
import franka_kinematics
from mpl_toolkits.mplot3d import Axes3D


franka_kin = franka_kinematics.FrankaKinematics()




def cluster_specifications(): 
	straw81 = np.array([0.44999826901475964, 0.10000205479846114, 1.0599142533194694, 4.244566712753927e-06])
	straw82 = np.array([0.44999851284989234, 0.0821539977781866, 0.9240894286207944, 0.3133357533444485])
	straw83 = np.array([0.4499984064530141, 0.0878313002183852, 0.9985176875440531, -0.32215970186415416])
	bunch_xyz = []
	bunch_col = []
	bunch_orient = []

	goal8c = True
	goal8_n1c = False
	goal8_n2c = False


	config = 8
	theta8 = 0 

	goal8o = np.array([0,0,1])  # important goal orientation definition
	goal8_n1o = np.array([0,0,1]) 
	goal8_n2o = np.array([0,0,1])


	g8_n1_so = (np.pi)/20 
	g8_n2_so = -(np.pi)/20


	bunch_xyz.append(straw81[0:3])
	bunch_xyz.append(straw82[0:3]) 
	bunch_xyz.append(straw83[0:3])

	bunch_col.append(goal8c)
	bunch_col.append(goal8_n1c)
	bunch_col.append(goal8_n2c)
	bunch_orient.append(goal8o)
	bunch_orient.append(goal8_n1o)
	bunch_orient.append(goal8_n2o)
	spherical_angles = [[g8_n1_so, 0], [g8_n2_so, 0]]
	Lstems = [0.28, 0.2, 0.13]

	return bunch_xyz, bunch_col, bunch_orient, spherical_angles, Lstems, config



def plot_traj(traj, cond_pts, xf, cam =None):
	ee_traj = []
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	for i in range(traj.shape[0]): 
		T, _ = franka_kin.fwd_kin(traj[i, :])
		ee_traj.append(T[:3,3])
		ax.scatter(ee_traj[i][0], ee_traj[i][1], ee_traj[i][2], c='b', marker='o')

	ax.scatter(ee_traj[100][0], ee_traj[100][1], ee_traj[100][2], s=100, c='k', marker='o')
	print('enter cluster at = ', traj[100])
	print('goal is = ', traj[-1])
	ax.scatter(xf[0], xf[1], xf[2], s = 100, c='r', marker='o')
	if cam != None:
		ax.scatter(cam[0], cam[1], cam[2], s = 100, c='c', marker='o')
	#counting = 0
	for Ip in range(len(cond_pts)):
		#if counting == 0:
		pt = cond_pts[Ip]
		ax.scatter(pt[0], pt[1], pt[2], s = 50, c='g', marker='o')

	ax.set_xlabel('X')
	ax.set_ylabel('Y')
	ax.set_zlabel('Z')
	plt.show()


def traj_sat(traj):
	J1 = [-2.89725, 2.89725]
	J2 = [-1.76278, 1.76278]
	J3 = [-2.89725, 2.89725]
	J4 = [-3.07178, 0.0873]
	J5 = [-2.89725, 2.89725]
	J6 = [-0.01745, 3.75246]
	J7 = [-2.89725, 2.89725]
	Jmin = [J1[0], J2[0], J3[0], J4[0], J5[0], J6[0], J7[0]]
	Jmax = [J1[1], J2[1], J3[1], J4[1], J5[1], J6[1], J7[1]]


	for i in range(traj.shape[0]): 
		for j in range(traj.shape[1]):
			if traj[i][j] < Jmin[j]:
				traj[i][j] = Jmin[j]
			elif traj[i][j] > Jmax[j]:
				traj[i][j] = Jmax[j]

	with open('traj_opt_sat.npz', 'w') as f1:
		np.save(f1, traj)


	with open('traj_opt_sat.csv', 'w') as f2:
		np.savetxt(f2, traj, delimiter=',', fmt='%f')

	return traj



def get_abspos(traj):
	goal = traj[-1]
	print('goal=', goal)
	T, T_joint = franka_kin.fwd_kin(goal)
	print('Tj shape=', T_joint.shape)
	T_goal = T_joint[-1]
	T01 = T_joint[0]
	T02 = T_joint[1]
	T03 = T_joint[2]
	T04 = T_joint[3]
	T05 = T_joint[4]
	T06 = T_joint[5]
	X_goal = T_goal[0:3,3]
	R_goal = T_goal[0:3, 0:3]
	rpy_goal = tf_tran.euler_from_matrix(R_goal, 'rxyz')
	quat_goal =	tf_tran.quaternion_from_matrix(T_goal)

	X_j1 = T01[0:3,3]
	rpy_j1 = tf_tran.euler_from_matrix(T01[0:3, 0:3], 'rxyz')

	X_j2 = T02[0:3,3]
	rpy_j2 = tf_tran.euler_from_matrix(T02[0:3, 0:3], 'rxyz')

	X_j3 = T03[0:3,3]
	rpy_j3 = tf_tran.euler_from_matrix(T03[0:3, 0:3], 'rxyz')

	X_j4 = T04[0:3,3]
	rpy_j4 = tf_tran.euler_from_matrix(T04[0:3, 0:3], 'rxyz')

	X_j5 = T05[0:3,3]
	rpy_j5 = tf_tran.euler_from_matrix(T05[0:3, 0:3], 'rxyz')

	X_j6 = T06[0:3,3]
	rpy_j6 = tf_tran.euler_from_matrix(T06[0:3, 0:3], 'rxyz')
	print('J1 goal=', [X_j1[0], X_j1[1], X_j1[2], rpy_j1[0], rpy_j1[1], rpy_j1[2]])
	print('J2 goal=', [X_j2[0], X_j2[1], X_j2[2], rpy_j2[0], rpy_j2[1], rpy_j2[2]])
	print('J3 goal=', [X_j3[0], X_j3[1], X_j3[2], rpy_j3[0], rpy_j3[1], rpy_j3[2]])
	print('J4 goal=', [X_j4[0], X_j4[1], X_j4[2], rpy_j4[0], rpy_j4[1], rpy_j4[2]])
	print('J5 goal=', [X_j5[0], X_j5[1], X_j5[2], rpy_j5[0], rpy_j5[1], rpy_j5[2]])
	print('J6 goal=', [X_j6[0], X_j6[1], X_j6[2], rpy_j6[0], rpy_j6[1], rpy_j6[2]])
	print('J7 goal=', [X_goal[0], X_goal[1], X_goal[2], rpy_goal[0], rpy_goal[1], rpy_goal[2]])



def get_abstraj(traj_sat):
	pose_j1 = []
	pose_j2 = []
	pose_j3 = []
	pose_j4 = []
	pose_j5 = []
	pose_j6 = []
	pose_j7 = []
	quat_norm = []

	for i in range(len(traj_sat)):
		config = traj_sat[i]
		T, T_joint = franka_kin.fwd_kin(config)

		T_j7 = T_joint[-1]
		T01 = T_joint[0]
		T02 = T_joint[1]
		T03 = T_joint[2]
		T04 = T_joint[3]
		T05 = T_joint[4]
		T06 = T_joint[5]
		X_j7 = T_j7[0:3,3]
		R_j7 = T_j7[0:3, 0:3]
		rpy_j7 = tf_tran.euler_from_matrix(R_j7, 'rxyz')
		quat_j7 =	tf_tran.quaternion_from_matrix(T_j7)
		norm_quat_j7 = np.linalg.norm(quat_j7)

		X_j1 = T01[0:3,3]
		rpy_j1 = tf_tran.euler_from_matrix(T01[0:3, 0:3], 'rxyz')
		quat_j1 =	tf_tran.quaternion_from_matrix(T01)
		norm_quat_j1 = np.linalg.norm(quat_j1)

		X_j2 = T02[0:3,3]
		rpy_j2 = tf_tran.euler_from_matrix(T02[0:3, 0:3], 'rxyz')
		quat_j2 = tf_tran.quaternion_from_matrix(T02)
		norm_quat_j2 = np.linalg.norm(quat_j2)

		X_j3 = T03[0:3,3]
		rpy_j3 = tf_tran.euler_from_matrix(T03[0:3, 0:3], 'rxyz')
		quat_j3 = tf_tran.quaternion_from_matrix(T03)
		norm_quat_j3 = np.linalg.norm(quat_j3)

		X_j4 = T04[0:3,3]
		rpy_j4 = tf_tran.euler_from_matrix(T04[0:3, 0:3], 'rxyz')
		quat_j4 = tf_tran.quaternion_from_matrix(T04)
		norm_quat_j4 = np.linalg.norm(quat_j4)

		X_j5 = T05[0:3,3]
		rpy_j5 = tf_tran.euler_from_matrix(T05[0:3, 0:3], 'rxyz')
		quat_j5 = tf_tran.quaternion_from_matrix(T05)
		norm_quat_j5 = np.linalg.norm(quat_j5)

		X_j6 = T06[0:3,3]
		rpy_j6 = tf_tran.euler_from_matrix(T06[0:3, 0:3], 'rxyz')
		quat_j6 = tf_tran.quaternion_from_matrix(T06)
		norm_quat_j6 = np.linalg.norm(quat_j6)

		#print('J1 =', [X_j1[0], X_j1[1], X_j1[2], rpy_j1[0], rpy_j1[1], rpy_j1[2]])
		#print('J2 =', [X_j2[0], X_j2[1], X_j2[2], rpy_j2[0], rpy_j2[1], rpy_j2[2]])
		#print('J3 =', [X_j3[0], X_j3[1], X_j3[2], rpy_j3[0], rpy_j3[1], rpy_j3[2]])
		#print('J4 =', [X_j4[0], X_j4[1], X_j4[2], rpy_j4[0], rpy_j4[1], rpy_j4[2]])
		#print('J5 =', [X_j5[0], X_j5[1], X_j5[2], rpy_j5[0], rpy_j5[1], rpy_j5[2]])
		#print('J6 =', [X_j6[0], X_j6[1], X_j6[2], rpy_j6[0], rpy_j6[1], rpy_j6[2]])
		#print('J7 =', [X_j7[0], X_j7[1], X_j7[2], rpy_j7[0], rpy_j7[1], rpy_j7[2]])
		pose_j1.append([X_j1[0], X_j1[1], X_j1[2], rpy_j1[0], rpy_j1[1], rpy_j1[2]])
		pose_j2.append([X_j2[0], X_j2[1], X_j2[2], rpy_j2[0], rpy_j2[1], rpy_j2[2]])
		pose_j3.append([X_j3[0], X_j3[1], X_j3[2], rpy_j3[0], rpy_j3[1], rpy_j3[2]])
		pose_j4.append([X_j4[0], X_j4[1], X_j4[2], rpy_j4[0], rpy_j4[1], rpy_j4[2]])
		pose_j5.append([X_j5[0], X_j5[1], X_j5[2], rpy_j5[0], rpy_j5[1], rpy_j5[2]])
		pose_j6.append([X_j6[0], X_j6[1], X_j6[2], rpy_j6[0], rpy_j6[1], rpy_j6[2]])
		pose_j7.append([X_j7[0], X_j7[1], X_j7[2], rpy_j7[0], rpy_j7[1], rpy_j7[2]])
		quat_norm.append([norm_quat_j1, norm_quat_j2, norm_quat_j3, norm_quat_j4, norm_quat_j5, norm_quat_j6, norm_quat_j7])

	with open('pose_j1_opt.csv', 'w') as f1:
		np.savetxt(f1, pose_j1, delimiter=',', fmt='%f')

	with open('pose_j2_opt.csv', 'w') as f2:
		np.savetxt(f2, pose_j2, delimiter=',', fmt='%f')

	with open('pose_j3_opt.csv', 'w') as f3:
		np.savetxt(f3, pose_j3, delimiter=',', fmt='%f')

	with open('pose_j4_opt.csv', 'w') as f4:
		np.savetxt(f4, pose_j4, delimiter=',', fmt='%f')

	with open('pose_j5_opt.csv', 'w') as f5:
		np.savetxt(f5, pose_j5, delimiter=',', fmt='%f')

	with open('pose_j6_opt.csv', 'w') as f6:
		np.savetxt(f6, pose_j6, delimiter=',', fmt='%f')

	with open('pose_j7_opt.csv', 'w') as f7:
		np.savetxt(f7, pose_j7, delimiter=',', fmt='%f')

	with open('quat_norm_opt.csv', 'w') as f8:
		np.savetxt(f8, quat_norm, delimiter=',', fmt='%f')


def check_limits():

	J1 = [-2.89725, 2.89725]
	J2 = [-1.76278, 1.76278]
	J3 = [-2.89725, 2.89725]
	J4 = [-3.07178, -0.0698]
	J5 = [-2.89725, 2.89725]
	J6 = [-0.01745, 3.75246]
	J7 = [-2.89725, 2.89725]


	Jmin = [J1[0], J2[0], J3[0], J4[0], J5[0], J6[0], J7[0]]
	Jmax = [J1[1], J2[1], J3[1], J4[1], J5[1], J6[1], J7[1]]

	data = np.load('/home/sariah/intProMP_franka/src/TrajOpt_UoL/promp_trajopt/traj_opt/src/results/bounds/latest/seqOpt_smooth0_obs1_max10_3cmsafety_20cmNeighdist/traj_init_opt.npz')
	print('data=', data.shape)
	traj_init = data[0]
	prompOpt = data[1]
	#print('prompOpt=', prompOpt.shape)


	for i in range(prompOpt.shape[0]): 
		for j in range(prompOpt.shape[1]):
			if prompOpt[i][j] < Jmin[j]:
				print("less min; sample, joint: {}, {}".format(i, j))
				print('traj opt= ', prompOpt[i][j])
				prompOpt[i][j] = Jmin[j]
			elif prompOpt[i][j] > Jmax[j]:
				print("more max; sample, joint: {}, {}".format(i, j))
				print('traj opt= ', prompOpt[i][j])
				prompOpt[i][j] = Jmax[j]


	with open('traj_opt_final_sat.npz', 'w') as f2:
		np.save(f2, prompOpt)

	with open('traj_opt_final_sat.csv', 'w') as f3:
		np.savetxt(f3, prompOpt, delimiter=',', fmt='%f')


if __name__ == "__main__":

	latest  = True 

	if latest == True:
		check_limits() 
	else: 

		data = np.load('./results/push_obs_thres_low/traj_init_opt.npz')
		print('data=', data.shape)
		traj_init = data[0]
		prompOpt = data[1]
		print('prompOpt=', prompOpt.shape)

		bunch_xyz, bunch_col, bunch_orient, spherical_angles, Lstems, config = cluster_specifications()
		cluster_init = bunch_xyz[0]
		cond_pts = bunch_xyz[1:]
		minZ = cond_pts[0]
		cam = [cluster_init[0], cluster_init[1], minZ[2]-0.1]


		plot_traj(prompOpt, cond_pts, cluster_init, cam = cam)
		traj_sat = traj_sat(prompOpt)
		get_abspos(traj_sat)
		get_abstraj(traj_sat)




# the 100th point is : [ 2.10655928,  0.44367142,  0.84542683,  0.42038999,  0.90584957, -0.04198944, -4.80451678]
# the last 200th point is : [-0.56878827, -0.23717107, -1.15477715,  0.26341949,  0.81212248, 1.21409272, -9.16115558])