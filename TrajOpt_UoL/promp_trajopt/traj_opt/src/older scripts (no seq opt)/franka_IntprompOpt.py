#!/usr/bin/env python

# import sys
# class KineticImportsFix:
# 	def __init__(self, kinetic_dist_packages="/opt/ros/kinetic/lib/python2.7/dist-packages"):
# 		self.kinetic_dist_packages = kinetic_dist_packages

# 	def __enter__(self):
# 		sys.path.remove(self.kinetic_dist_packages)

# 	def __exit__(self, exc_type, exc_val, exc_tb):
# 		sys.path.append(self.kinetic_dist_packages)


#with KineticImportsFix():   
import os
import sys
import rospy
import numpy as np
from operator import itemgetter
import scipy.optimize as opt
from scipy.optimize import LinearConstraint
from scipy.optimize import Bounds
from traj_opt import trajectory_optimization    
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import basis as basis
import promps as promps
import phase as phase
import tf.transformations as tf_tran
import matplotlib.pyplot as plt
import franka_kinematics
from mpl_toolkits.mplot3d import Axes3D
#from promp_uol import Franka_pushing_clusters as franka_decluster
import Franka_pushing_clusters as franka_decluster


franka_kin = franka_kinematics.FrankaKinematics()
franka_actions = franka_decluster.actions()

#np_load_old = np.load

## modify the default parameters of np.load
#np.load = lambda *a,**k: np_load_old(*a, allow_pickle=True, **k)


# not latest in use, it does not call the pushing cost
class promp_pub():

	def __init__(self):
		#rospy.init_node('franka_intpromp_publisher', anonymous=True)
		#self.rate = rospy.Rate(50)
		self.q0 = np.array([])
		self.q0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3]
		self.Tf = 0.3
		self.T0 = 0
		self.t1samples = 100
		self.t2samples = 100
		self.init_cov = np.eye(len(self.q0))  *0.000001
		#self.pubTrajD = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
		#self.subJState  = rospy.Subscriber("/joint_states", JointState, callback=self.StatesCallback)

	def StatesCallback(self,msg):
		name = msg.name
		self.q0 = msg.position[2:]
		print('q0=', self.q0)
		self.qdot0 = msg.velocity[2:]
		if (len(self.q0) != 0):
			self.subJState.unregister()

	def jointTrajectoryCommand(self,traj, t=1):
		jt = JointTrajectory()
		jt.header.stamp = rospy.Time.now()
		jt.header.frame_id = "panda_arm"
		jt.joint_names.append("panda_arm_joint1")
		jt.joint_names.append("panda_arm_joint2")
		jt.joint_names.append("panda_arm_joint3")
		jt.joint_names.append("panda_arm_joint4")
		jt.joint_names.append("panda_arm_joint5")
		jt.joint_names.append("panda_arm_joint6")
		jt.joint_names.append("panda_arm_joint7")
		J1 = traj[:, 0]
		J2 = traj[:, 1]
		J3 = traj[:, 2]
		J4 = traj[:, 3]
		J5 = traj[:, 4]
		J6 = traj[:, 5]
		J7 = traj[:, 6]

		n = len(J1)
		dt = np.linspace(float(t)/n, t+float(t)/n, n) #added float before t to avoid future division

		for i in range (n):
			p = JointTrajectoryPoint()
			p.positions.append(J1[i])
			p.positions.append(J2[i])
			p.positions.append(J3[i])
			p.positions.append(J4[i])
			p.positions.append(J5[i])
			p.positions.append(J6[i])
			p.positions.append(J7[i])

			p.time_from_start = rospy.Duration.from_sec(dt[i])  # time_from_start is the point in time at which that TrajectoryPoint should be executed.

			jt.points.append(p)
		self.pubTrajD.publish(jt)
		#time.sleep(1)
		del p.positions[:]
		del jt.points[:]


	def load_demos(self,): 
		# Refer to simple_example.py in folder promp/examples/python_promp/simple_example.py, for comments on functions (simple_example.py and franka_promp.py are similar scripts)
		with open('/home/sariah/intProMP_franka/src/TrajOpt_UoL/promp_trajopt/traj_opt/src/100demos.npz', 'r') as f:
		    data = np.load(f, allow_pickle=True)
		    self.Q = data['Q']#[:97]
		    self.time = data['time']#[:97] # demo 98 has less than 30 samples
		Q_row = self.Q.shape

		self.timelist1 = [] 
		self.Qlist1 = []
		self.timelist2 = [] 
		self.Qlist2 = []
		loop = 0
		for item in self.Q:
			self.Qlist1.append(item[:10,:])
			self.Qlist2.append(item[10:,:])
			#loop += 1
			#print('loop=', loop)
		#print('Q row=', Q_row) # nb of demos (104) x time samples (162) x nb of DoF (7) 
		for item in self.time:
			self.timelist1.append(item[:10])
			self.timelist2.append(item[10:])

		#print('time at 30s=', self.timelist2[0])
		

		## restore np.load for future normal usage
		#np.load = np_load_old

		################################################
		# To plot demonstrated end-eff trajectories

		fig = plt.figure()
		ax = fig.add_subplot(111, projection='3d')
		for i in range(len(self.Q)):
		    endEffTraj = franka_kin.fwd_kin_trajectory(self.Q[i])
		    ax.scatter(endEffTraj[:,0], endEffTraj[:,1], endEffTraj[:,2], c='b', marker='.')
		plt.title('EndEff')
		plt.show()

		return self.Q, self.time
		######################################
		# To plot demonstrated trajectories Vs time

		# for plotDoF in range(7):
		#     plt.figure()
		#     for i in range(len(Q)):
		#         plt.plot(time[i] - time[i][0], Q[i][:, plotDoF])

		#     plt.title('DoF {}'.format(plotDoF))
		# plt.xlabel('time')
		# plt.title('demonstrations')

		############################################
	def promp_generator(self, IC, goal):

		phaseGenerator = phase.LinearPhaseGenerator()
		basisGenerator1 = basis.NormalizedRBFBasisGenerator(phaseGenerator, numBasis=4, duration=0.1, basisBandWidthFactor=0.1, # check duration
		                                                   numBasisOutside=1)
		basisGenerator2 = basis.NormalizedRBFBasisGenerator(phaseGenerator, numBasis=4, duration=self.Tf, basisBandWidthFactor=3,
		                                                   numBasisOutside=1)

		time_normalised1 = np.linspace(self.T0, 0.1, self.t1samples)
		time_normalised2 = np.linspace(0.1, self.Tf, self.t2samples)
		time_normalised = np.concatenate((time_normalised1, time_normalised2), axis=0)
		nDof = 7
		proMP1 = promps.ProMP(basisGenerator1, phaseGenerator, nDof)
		proMP2 = promps.ProMP(basisGenerator2, phaseGenerator, nDof)
		plotDof = 2

		################################################################
		# Conditioning in JointSpace

		desiredTheta = np.array([0.5, 0.7, 0.5, 0.2, 0.6, 0.8, 0.1])
		desiredVar = np.eye(len(desiredTheta)) * 0.0001
		meanTraj, covTraj = proMP2.getMeanAndCovarianceTrajectory(time_normalised2)
		newProMP2 = proMP2.jointSpaceConditioning(self.Tf, desiredTheta=desiredTheta, desiredVar=desiredVar)
		traj_seq1 = proMP1.getTrajectorySamples(time_normalised1, 1)
		#print('traj_seq1 shape=', traj_seq1.shape)
		traj_seq2 = newProMP2.getTrajectorySamples(time_normalised2, 1)
		trajectories = np.concatenate((traj_seq1 ,traj_seq2), axis = 0)
		print('traj =', trajectories.shape)

		#plt.figure()
		#plt.plot(time_normalised, trajectories[:, plotDof, :])
		#plt.xlabel('time')
		#plt.title('Joint-Space conditioning for joint 2')

		proMP1.plotProMP(time_normalised1, [3, 4]) # refer to plotter.py, it plots means ans 2*std filled curves of newpromp, indices = [3, 4] refer to joints to plot
		newProMP2.plotProMP(time_normalised2, [3, 4]) ## put them later on same graph


		##################################################
		# Conditioning in Task Space

		learnedProMP1 = promps.ProMP(basisGenerator1, phaseGenerator, nDof) # regression model initialization
		learnedProMP2 = promps.ProMP(basisGenerator2, phaseGenerator, nDof) # regression model initialization
		learner1 = promps.MAPWeightLearner(learnedProMP1) # weights learning model
		learner2 = promps.MAPWeightLearner(learnedProMP2) 
		#print('q', Q[:][:120])
		learner1.learnFromData(self.Qlist1, self.timelist1) # get weights
		learner2.learnFromData(self.Qlist2, self.timelist2) 
		learned_promp1 = learnedProMP1.getTrajectorySamples(time_normalised1, 1)
		learned_promp2 = learnedProMP2.getTrajectorySamples(time_normalised1, 1)
		jtrajectories_learned = np.concatenate((learned_promp1, learned_promp2), axis=0)
		ttrajectories_learned = franka_kin.fwd_kin_trajectory(jtrajectories_learned)
		mu_theta, sig_theta = learnedProMP2.getMeanAndCovarianceTrajectory(np.array([1.0])) # get mean cov of the learnt promp in joint space at time T = 1s
		#print('mu_theta=', np.squeeze(mu_theta))
		#print('sig_theta=', np.squeeze(sig_theta))
		sig_theta = np.squeeze(sig_theta)
		mu_x = goal # desired mean ee pose/position at T = 1s, old= [0.6, 0.5, 0.8]
		sig_x = np.eye(3) * 0.0000002 # desired cov of ee pose/position at T = 1s
		q_home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3]
		T_desired, tmp = franka_kin.fwd_kin(q_home)
		mu_ang_euler_des = tf_tran.euler_from_matrix(T_desired, 'szyz')
		sig_euler = np.eye(3) * 0.0002
		print('starting IK')
		post_mean_Ash, post_cov = franka_kin.inv_kin_ash_pose(np.squeeze(mu_theta), sig_theta, mu_x, sig_x, mu_ang_euler_des, sig_euler) # output is in joint space, desired mean and cov
		#print('post_mean =',post_mean_Ash.shape) 7x1
		#print('post_cov =',post_cov.shape) 7x7
		print('finishing IK')
		newProMP1 = learnedProMP1.jointSpaceConditioning(self.T0, desiredTheta= self.q0, desiredVar=self.init_cov)
		newProMP2 = learnedProMP2.jointSpaceConditioning(self.Tf, desiredTheta= post_mean_Ash, desiredVar=post_cov)
		trajectories_task1_conditioned = newProMP1.getTrajectorySamples(time_normalised1, 1)
		trajectories_task2_conditioned = newProMP2.getTrajectorySamples(time_normalised2, 1)
		trajectories_task_conditioned = np.concatenate((trajectories_task1_conditioned, trajectories_task2_conditioned), axis=0)
		ttrajectories_task_conditioned = franka_kin.fwd_kin_trajectory(trajectories_task_conditioned)

		# Get orientation of EE at T = 1s from demonstrations
		q_final = mu_theta
		q_final = np.squeeze(q_final)
		print('q_final=', q_final)
		T_final, tmp_final = franka_kin.fwd_kin(q_final)
		mu_ang_euler_final = tf_tran.euler_from_matrix(T_final, 'szyz')

		return [newProMP1, newProMP2], ttrajectories_task_conditioned, [time_normalised1, time_normalised2], nDof, mu_ang_euler_final, sig_euler, ttrajectories_learned


	def optCallback(self, xk, state):   # xk is joints traj, here we calculate total costs based on traj of joints
        # xk = xk.reshape(len(xk) / 7, 7)
		print("xk size: {}\n".format(len(xk)))
		costs = self.calculate_total_cost(xk) # xk as traj wll be nxlen(body pts)
		self.optCurve.append(xk) # xk is the result of each optimization problem iteration
		print("Iteration {}: {}\n".format(len(self.optCurve), costs))  # first {} refers to first element of format() and second {} refers to second element



	def linear_constraints(self, trajectoryFlat, condpts, tcond, mean, cov):
		# Assign a timestamp for each condpt
		lc= []
		lb = []
		ub = []
		#trajectory = np.squeeze(trajectory)
		print('trajflat shape for LC=', trajectoryFlat.shape)
		for cd in range(len(condpts)):
			tcond_disc = str(np.round(tcond[cd]*(self.t1samples+self.t2samples)/self.Tf,0))
			tcond_disc = tcond_disc.rstrip('0').rstrip('.') if '.' in tcond_disc else tcond_disc
			#print('traj at tcond=',trajectory[int(tcond_disc)])
			## If constraints are in task space, do the following:
			#T, _ = franka_kin.fwd_kin([trajectoryFlat[(franka_kin.numJoints-6)*int(tcond_disc)], trajectoryFlat[(franka_kin.numJoints-5)*int(tcond_disc)], trajectoryFlat[(franka_kin.numJoints-4)*int(tcond_disc)], trajectoryFlat[(franka_kin.numJoints-3)*int(tcond_disc)], trajectoryFlat[(franka_kin.numJoints-2)*int(tcond_disc)], trajectoryFlat[(franka_kin.numJoints-1)*int(tcond_disc)], trajectoryFlat[(franka_kin.numJoints)*int(tcond_disc)]])
			#pos = T[0:3,3]
			#quat = tf_tran.quaternion_from_matrix( T )
			#ee_pose = np.hstack( (pos, quat) )

			## If constraints are in operational space, do the following:	
			print(('traj shape=',len(trajectoryFlat[(franka_kin.numJoints-6)*int(tcond_disc)+1:(franka_kin.numJoints-5)*int(tcond_disc)-1])))
			print('zeros matix shape=',(np.zeros((franka_kin.numJoints-5)*int(tcond_disc)-1 -(franka_kin.numJoints-6)*int(tcond_disc)).shape))
			pos_aug = np.concatenate((np.zeros((franka_kin.numJoints-6)*int(tcond_disc)-1)*trajectoryFlat[0:(franka_kin.numJoints-6)*int(tcond_disc)-1], trajectoryFlat[(franka_kin.numJoints-6)*int(tcond_disc)], np.zeros((franka_kin.numJoints-5)*int(tcond_disc)-1 -(franka_kin.numJoints-6)*int(tcond_disc))*trajectoryFlat[(franka_kin.numJoints-6)*int(tcond_disc)+1:(franka_kin.numJoints-5)*int(tcond_disc)-1], trajectoryFlat[(franka_kin.numJoints-5)*int(tcond_disc)], np.zeros((franka_kin.numJoints-4)*int(tcond_disc)-1 -(franka_kin.numJoints-5)*int(tcond_disc))*trajectoryFlat[(franka_kin.numJoints-5)*int(tcond_disc)+1:(franka_kin.numJoints-4)*int(tcond_disc)-1], trajectoryFlat[(franka_kin.numJoints-4)*int(tcond_disc)], np.zeros((franka_kin.numJoints-3)*int(tcond_disc)-1 -(franka_kin.numJoints-4)*int(tcond_disc))*trajectoryFlat[(franka_kin.numJoints-4)*int(tcond_disc)+1:(franka_kin.numJoints-3)*int(tcond_disc)-1], trajectoryFlat[(franka_kin.numJoints-3)*int(tcond_disc)], np.zeros((franka_kin.numJoints-2)*int(tcond_disc)-1 -(franka_kin.numJoints-3)*int(tcond_disc))*trajectoryFlat[(franka_kin.numJoints-3)*int(tcond_disc)+1:(franka_kin.numJoints-2)*int(tcond_disc)-1], trajectoryFlat[(franka_kin.numJoints-2)*int(tcond_disc)], np.zeros((franka_kin.numJoints-1)*int(tcond_disc)-1 -(franka_kin.numJoints-2)*int(tcond_disc))*trajectoryFlat[(franka_kin.numJoints-2)*int(tcond_disc)+1:(franka_kin.numJoints-1)*int(tcond_disc)-1], trajectoryFlat[(franka_kin.numJoints-1)*int(tcond_disc)], np.zeros((franka_kin.numJoints)*int(tcond_disc)-1 - (franka_kin.numJoints-1)*int(tcond_disc)+1  )*trajectoryFlat[(franka_kin.numJoints-1)*int(tcond_disc)+1:(franka_kin.numJoints)*int(tcond_disc)-1] , trajectoryFlat[(franka_kin.numJoints)*int(tcond_disc)]), np.zeros(len(trajectoryFlat) - (franka_kin.numJoints)*int(tcond_disc))*trajectoryFlat[(franka_kin.numJoints)*int(tcond_disc)+1:], axis=None) 
			#pos_aug = np.concatenate((trajectoryFlat[(franka_kin.numJoints-6)*int(tcond_disc)], trajectoryFlat[(franka_kin.numJoints-5)*int(tcond_disc)], trajectoryFlat[(franka_kin.numJoints-4)*int(tcond_disc)], trajectoryFlat[(franka_kin.numJoints-3)*int(tcond_disc)], trajectoryFlat[(franka_kin.numJoints-2)*int(tcond_disc)], trajectoryFlat[(franka_kin.numJoints-1)*int(tcond_disc)], trajectoryFlat[(franka_kin.numJoints)*int(tcond_disc)]), axis=None)

			lc.append(pos_aug)
			print('pose shape=', pos_aug.shape)
			lb.append(mean[cd]) 
			print('lb shape=', lb[0].shape)
			ub.append(mean[cd]) 
			print('finish appending')
			#LC.extend([-condpts[cd], condpts[cd]])
		return LinearConstraint(lc, lb, ub)
		#return LinearConstraint(LC[0], LC[1:])


	## Start constrained optimization
	def intpromp_opt(self, IntProMP, cond_pts, tcond, mean, cov):

		traj_opt = trajectory_optimization()
		#trajectory = np.load('IntProMP_Pushing.npz')
		trajectory = IntProMP # 200x7x1
		#trajectory= np.squeeze(trajectory) # 200x7
		b, r , c= trajectory.shape  # b: blocks = 100 (time), r = rows(7, dof) and c = columns (20, samples from the Gaussian distribution at each dof at an instant of time)
		print('traj dim is: {}, {}, {}'.format(b, r, c))
		#print('ProMP traj is: {}'.format(trajectory))
	 
		traj_size = len(trajectory) 
		print('size: {}'.format(traj_size))

		initial_joint_values = trajectory[0,:,:]
		desired_joint_values = trajectory[(traj_size - 1),:,:]
		initial_trajectory = trajectory # at time 0
		trajectoryFlat = trajectory.T.reshape(-1) # by transpose: we got 20x7x100, by reshape(-1) we merge the elements of the original array in one line
		bf = trajectoryFlat.shape
		print('probabilistic_trajflat dim is: {}'.format(bf))

		traj_mean = np.mean(trajectory, axis=2)
		bm, rm = traj_mean.shape
		print('mean proba traj dim is: {}, {}'.format(bm, rm)) 
		obst_cost_grad_analytic = np.reshape(traj_opt.calculate_obstacle_cost_gradient(trajectoryFlat), -1)
		#print('obst_cost_grad_analytic {} \n'.format(obst_cost_grad_analytic))  # gradient cost of initialized traj 

		################### With Gradient ############################
	    # minimize total cost
	    # scipy.minimize(fun, x0, args=(), method='BFGS', jac=None, tol=None, callback=None, options={'gtol': 1e-05, 'norm': inf, 'eps': 1.4901161193847656e-08, 'maxiter': None, 'disp': False, 'return_all': False})
	    # BFGS solver uses a limited computer memory and it is a popular algorithm for parameter estimation in machine learning.
	    # disp : Set to True to print convergence messages.
	    # mixter :max nb of iterations
	    # gtol : Gradient norm must be less than gtol before successful termination.

		LC = self.linear_constraints(trajectoryFlat, cond_pts, tcond, mean, cov)
		#optimized_trajectory = opt.minimize(traj_opt.calculate_total_cost, trajectoryFlat, method='BFGS', jac=traj_opt.cost_gradient_analytic, options={'maxiter': 15, 'disp': True}, callback=traj_opt.optim_callback)
		optimized_trajectory = opt.minimize(traj_opt.calculate_total_cost, trajectoryFlat, method='trust-constr', jac=traj_opt.cost_gradient_analytic, constraints = LC, options={'maxiter': 15, 'disp': True}, callback=self.optCallback)
		print('passed the optimization, opt traj=', optimized_trajectory)
		optimized_trajectory = np.transpose(optimized_trajectory.x.reshape((7, len(optimized_trajectory.x) / 7)))
		optimized_trajectory = np.insert(optimized_trajectory, 0, initial_joint_values, axis=0)
		optimized_trajectory = np.insert(optimized_trajectory, len(optimized_trajectory), desired_joint_values, axis=0)
		print('result of optimizaed trajectory {}\n'.format(optimized_trajectory))
		traj_opt.animation(optimized_trajectory, initial_trajectory)

		print('Finished plotting the optimized trajectory with Gradient passed to the minimization Algo')

		print('finished')





def promp_plotter(time_normalised, trajectories_task_conditioned):
	plt.figure()
	plt.plot(time_normalised, trajectories_task_conditioned[:, plotDof, :]) # 10 samples in joint space for joint : plotDof
	plt.xlabel('time')
	plt.title('Task-Space conditioning for joint 2')

	##############################################
	# Plot of end-effector trajectories

	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	for i in range(trajectories_task_conditioned.shape[2]): # shape[2] is time samples
	    endEffTraj = franka_kin.fwd_kin_trajectory(trajectories_task_conditioned[:, :, i])
	    ax.scatter(endEffTraj[:, 0], endEffTraj[:, 1], endEffTraj[:, 2], c='b', marker='.')
	plt.xlabel('X')
	plt.ylabel('Y')
	plt.title('EE trajectories after task space Conditioning')
	##############################################
	plt.show()
	print('Finished')



##################################################
def promp_saver(trajectories_task_conditioned):
	# To save the task conditioned trajectories for playing back on robot

	with open('traject_task_conditioned1.npz', 'w') as f:
	    np.save(f, trajectories_task_conditioned)


##################################################
def cluster_request():
	bunch_xyz = []
	bunch_col = []
	bunch_orient = []

	# Collect point clouds
	X_goal7 = np.array([0, 0.5, 0.46-0.13])
	X_goal8 = np.array([0, 0.5, 0.46-0.13]) 
	X_goal9 = np.array([-0.08+0.2*np.sin((np.pi)/10), 0.5, 0.46-0.2*np.cos((np.pi)/10)])   

	# neighbors of X_goal1
	goal7_n1 = np.array([0.1-0.23*np.sin((np.pi)/6), X_goal7[1], 0.46-0.23*np.cos((np.pi)/6)])  	

	# neighbors of X_goal2
	goal8_n1 = np.array([0.12-0.28*np.sin((np.pi)/6), X_goal8[1], 0.46-0.28*np.cos((np.pi)/6)]) 
	goal8_n2 = np.array([-0.1+0.2*np.sin((np.pi)/8), X_goal8[1], 0.46-0.2*np.cos((np.pi)/8)])  

	# neighbors of X_goal3
	goal9_n1 = np.array([0.08-0.28*np.sin((np.pi)/8), 0.5, 0.46-0.28*np.cos((np.pi)/8)])  
	goal9_n2 = np.array([0, 0.5, 0.46-0.13])


	# colors
	X_goal7c = True #  is a mature fruit, red
	X_goal8c = True
	X_goal9c = True
	goal7_n1c = False
	goal8_n1c = False
	goal8_n2c = False
	goal9_n1c = False
	goal9_n2c = False


	# orientation: rotation about y-axis of camera2 (mobile wrt arm base) , or about z-axis of camera1 (fixed wrt arm base) 
	# the following are preset orientation , call form.get_orientation when importing pcls
	theta7 = 0 
	theta7n1 = (np.pi)/6 
	theta8 = 0 
	theta9 = (np.pi)/10
	#X_goal7o = np.array([0,0,1]) # important goal orientation definition
	X_goal7o = np.array([0,0,1]) 
	X_goal8o = np.array([0,0,1])  # important goal orientation definition
	X_goal9o = np.cos(theta9)* np.array([0,0,1])  # important goal orientation definition
	goal7_n1o = np.cos(theta7n1)* np.array([0,0,1])
	goal8_n1o = np.array([0,0,1]) 
	goal8_n2o = np.array([0,0,1])
	goal9_n1o = np.array([0,0,1]) 
	goal9_n2o = np.array([0,0,1])  



	#print('Please enter which cluster configuration you want to simulate.')
	#print('case 7 is configuration where NF is below goal.')
	#print('case 8 is configuration where NF are below goal.')
	#print('case 9 is configuration where NF are @ below and above goal.')
	#conf = input()
	conf = 8
	if conf == 7:
		bunch_xyz.append(X_goal7) 
		bunch_xyz.append(goal7_n1)
		bunch_col.append(X_goal7c)
		bunch_col.append(goal7_n1c)
		bunch_orient.append(X_goal7o)
		bunch_orient.append(goal7_n1o)
	elif conf == 8:
		bunch_xyz.append(X_goal8)
		bunch_xyz.append(goal8_n1)
		bunch_xyz.append(goal8_n2)
		bunch_col.append(X_goal8c)
		bunch_col.append(goal8_n1c)
		bunch_col.append(goal8_n2c)
		bunch_orient.append(X_goal8o)
		bunch_orient.append(goal8_n1o)
		bunch_orient.append(goal8_n2o)
	elif conf == 9:
		bunch_xyz.append(X_goal9) 
		bunch_xyz.append(goal9_n1)
		bunch_xyz.append(goal9_n2)
		bunch_col.append(X_goal9c)
		bunch_col.append(goal9_n1c)
		bunch_col.append(goal9_n2c)
		bunch_orient.append(X_goal9o)
		bunch_orient.append(goal9_n1o)
		bunch_orient.append(goal9_n2o)  
	else:
		print('configuration requested doesnt exist')

	return bunch_xyz, bunch_col, bunch_orient, conf



def pushing_generator(GoalCond_promp, Goalpromp_Sampled, cond_pts,time_normalised, nDof, mu_ang_euler_final, sig_euler):
	# Get the min Z in the cluster 
	Goal = Goalpromp_Sampled[-1]
	minZ = Goal[2]
	for lowest in cond_pts:
		if lowest[2] < minZ:
			minZ = lowest[2]

	coord_s1 = []

	## random time vector, since we didn't collected data
	#tff1 = np.linspace(0,1, 152)
	#tff1 = np.repeat(np.array([tff1]), sdemo, axis = 0)

	#tff2 = np.linspace(0,1, 10)
	#tff2 = np.repeat(np.array([tff2]), sdemo, axis = 0)

	t0 = time_normalised[0][0]
	tf = time_normalised[1][-1]
	t_cam2 = time_normalised[0][-1]
	# To* design a systematic approach for the time vector to condition at
	#if len(cond_pts)/2 >= 2:
	#	t_cam2 = 0.5/(len(cond_pts)/2)
	#elif (len(cond_pts)/2) < 2 and (len(cond_pts)/2) != 0:
	#	t_cam2 = 0.5/(len(cond_pts)/2)
	#else:
	#	t_cam2 = 0.98

	#print('t_cam2=', t_cam2)
	#time_normalised1 = np.linspace(0, t_cam2, (2/2)*100)  # 1sec duration 
	#time_normalised2 = np.linspace(t_cam2, tf, (2/2)*100)  # 1sec duration 

	cond_pts_reduced = []

	## approach 1: to check if there is topset for the goal
	for i in range(0,len(cond_pts), 2):
		cond_pts_reduced.append(cond_pts[i])

	goal = np.array([Goal[0], Goal[1], Goal[2]+0.016])
	cluster_init = np.array([Goal[0], Goal[1], Goal[2]-0.016])

	#print('clusterinit=', cluster_init)

	start = []
	start.append(IC0)
	start.append(cluster_init)

	mu_x_IC = start[0]  

	# Conditioning at 1st goal point (same performance if cond at tf comes at end)
	print('cond_pts_reduced=', cond_pts_reduced)
	j = []

	if len(goal_update)!= 0:
		mu_x_tf = goal_update

	elif len(j) == 0 and len(goal_update)== 0: 
		print('i didnt get goal below nb')
		mu_x_tf = cluster_init
	else:
		inex = np.where(np.sum(np.abs(np.asarray(cond_pts) - np.asarray(cond_pts_reduced)[j[-1]]), axis = -1) == 0)
		inex = inex[-1][0]
		mu_x_tf = cond_pts[inex+1]


	#######################################################################################################################################

	# Conditioning at cluster bottom point :
	mu_theta1, sig_theta1 = GoalCond_promp[0].getMeanAndCovarianceTrajectory(np.array([t_cam2])) # get mean cov of the learnt promp in joint space at time T = 1s
	sig_theta1 = np.squeeze(sig_theta1)
	mu_x_t1g1 = [cluster_init[0], cluster_init[1], (minZ-0.1)]  # desired mean ee pose/position at T = 1s
	sig_x_t1g1 = np.eye(3) * 0.0000002 # desired cov of ee pose/position at T = 1s
	post_mean_push, post_cov_push = franka_kin.inv_kin_ash_pose(np.squeeze(mu_theta1), sig_theta1, mu_x_t1g1, sig_x_t1g1, mu_ang_euler_final, sig_euler) # output is in joint space, desired mean and cov
	print('finishing IK')
	newProMP1 = GoalCond_promp[0].jointSpaceConditioning(t_cam2, desiredTheta=post_mean_push, desiredVar=post_cov_push)
	newProMP2 = GoalCond_promp[1].jointSpaceConditioning(t_cam2, desiredTheta=post_mean_push, desiredVar=post_cov_push)
	trajectories_task1_conditioned = newProMP1.getTrajectorySamples(time_normalised[0], 1)
	trajectories_task2_conditioned = newProMP2.getTrajectorySamples(time_normalised[1], 1)
	trajectories_task_conditioned = np.concatenate((trajectories_task1_conditioned, trajectories_task2_conditioned), axis=0)

	#######################################################################################################################################


	## Pushing close neighbours
	mu_x_tfn = []
	if len(j) == 0 and len(goal_update)==0:
		for wpt in cond_pts:
			mu_x_tfn.append(wpt)
	elif len(j) == 0 and len(goal_update)!=0:
		for wpt in cond_pts:
			mu_x_tfn.append(wpt)
		mu_x_tfn.append(cluster_init)
	else:
		mu_x_tfn.append(cluster_init)
		cond_pts_sub = np.delete(np.asarray(cond_pts),-1, axis = 0) # to change the -1 with a systematic code
		print('cond_pts_sub=', cond_pts_sub)
		for el in cond_pts_sub:
			mu_x_tfn.append(el)

		
	t_discrete = np.linspace(t_cam2, tf, len(mu_x_tfn)+1,endpoint=False) # len(mu_x_tfn)
	#t_discrete = [0.1, 0.13, 0.16, 0.2, 0.25, 0.3]
	print('t_discrete=', t_discrete)
	print('mu_x_tfn=', len(mu_x_tfn))
	t_effec = []
	mean_push = []
	cov_push = []

	traj_cond = []
	traj_cond.append(newProMP2)

	for k in range(1,len(mu_x_tfn)+1): # range(0,len(mu_x_tfn))
		## Nonlinear scale of t_discrete : exponential one
		tn = t_discrete[k]
		if (k % 2) != 0:
			tn = t_discrete[k] - t_discrete[k] * 0.0 # Tune those for trajectory variation minimization, + t_discrete[k+1] * 0.1 only add it in simulation to increase time to reach 1st cond point
		elif k==(len(mu_x_tfn)):
			tn = t_discrete[k] - t_discrete[k] * 0.1 # not much diff if 0.0
		else:
			tn = t_discrete[k] - t_discrete[k] * 0.1 # not much diff if 0.0
		#tn = t_discrete[k]
		t_effec.append(tn)
		print('tn=', tn)
		mu_thetan, sig_thetan = traj_cond[k-1].getMeanAndCovarianceTrajectory(np.array([tn])) # get mean cov of the learnt promp in joint space at time T = 1s
		sig_thetan = np.squeeze(sig_thetan)
		mu_x_tn = mu_x_tfn[k-1] 
		#print('mu_x=',mu_x_tng1)
		sig_x_tn = np.eye(3) * 0.0000001 # desired cov of ee pose
		post_mean_push, post_cov_push = franka_kin.inv_kin_ash_pose(np.squeeze(mu_thetan), sig_thetan, mu_x_tn, sig_x_tn, mu_ang_euler_final, sig_euler) # output is in joint space, desired mean and cov
		mean_push.append(post_mean_push)
		cov_push.append(post_cov_push)
		newProMPn = traj_cond[k-1].jointSpaceConditioning(tn, desiredTheta=post_mean_push, desiredVar=post_cov_push)
		trajectories_taskn_conditioned = newProMPn.getTrajectorySamples(time_normalised[1], 1)

		traj_cond.append(newProMPn)

	## Save data for testing
  	jtrajectories_pushed = np.concatenate((trajectories_task1_conditioned,trajectories_taskn_conditioned), axis = 0)
	print('traj pushed jt=', jtrajectories_pushed.shape) 
	ttrajectories_pushed = franka_kin.fwd_kin_trajectory(np.squeeze(jtrajectories_pushed))
	print('traj pushed task=', ttrajectories_pushed.shape) 

	with open('jIntProMP_Pushing.npz', 'w') as f:
		np.save(f, jtrajectories_pushed)
	return jtrajectories_pushed, ttrajectories_pushed, cond_pts, t_effec, mu_x_t1g1, mean_push, cov_push



def plot_traj(traj, cond_pts, x0, xf, cam =None):

	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.scatter(traj[:,0], traj[:,1], traj[:,2], c='b', marker='.')
	ax.scatter(x0[0], x0[1], x0[2], s = 100, c='y', marker='o')
	ax.scatter(xf[0], xf[1], xf[2], s = 100, c='r', marker='o')
	if cam != None:
		ax.scatter(x_cam[0], x_cam[1], x_cam[2], s = 100, c='c', marker='o')
	#counting = 0
	for pt in cond_pts:
		#if counting == 0:
		ax.scatter(pt[0], pt[1], pt[2], s = 100, c='g', marker='o')
			#counting += 1
	ax.set_xlabel('X')
	ax.set_ylabel('Y')
	ax.set_zlabel('Z')
	plt.title('interpromp')
	plt.show()


	##################
	#Plotting
	# loop = 0
	# fig = plt.figure()
	# ax = fig.add_subplot(111, projection='3d')
	# ax.scatter(traj_Xee_condtn[-1], traj_Yee_condtn[-1], traj_Zee_condtn[-1], c='b', marker='.')
	# if (len(Ncond_pts) != 0): # plot subpts that are not conditioned (by optimization)
	# 	for i in range(len(Ncond_pts)):
	# 		Ncond_pt = Ncond_pts[i]
	# 		print('Ncond_pt =', Ncond_pt )
	# 		ax.scatter(Ncond_pt[0], Ncond_pt[1], Ncond_pt[2], s = 100, c='r', marker='o')
	# 		ax.scatter(np.repeat(Ncond_pt[0], 80), np.repeat(Ncond_pt[1], 80), np.linspace(Ncond_pt[2], Ncond_pt[2]+0.2, 80), c='g', marker='.')
	# for l in range(0, len(mu_x_tfn)):
	# 	mu_x_tng1 = mu_x_tfn[l]
	# 	print('mu_x_tng1=',mu_x_tng1)
	# 	if len(j) == 0 and len(goal_update)==0:
	# 		if (loop % 2 != 0):
	# 			ax.scatter(mu_x_tng1[0], mu_x_tng1[1], mu_x_tng1[2], s = 100, c='g', marker='o')
	# 		else:
	# 			ax.scatter(mu_x_tng1[0], mu_x_tng1[1], mu_x_tng1[2], s = 100, c='r', marker='o')
	# 		##### STEMS DRAWINGS
	# 		#ST_g1n
	# 		if (loop % 2 == 0):
	# 			print('loop is even')
	# 			ax.scatter(np.repeat(mu_x_tng1[0], 80), np.repeat(mu_x_tng1[1], 80), np.linspace(mu_x_tng1[2], mu_x_tng1[2]+0.2, 80), c='g', marker='.')
	# 		else:
	# 			print('loop is odd')
	# 			mu_x_tng1_prev = mu_x_tfn[l-1]
	# 			root = np.array([mu_x_tng1_prev[0], mu_x_tng1_prev[1], mu_x_tng1_prev[2]+0.2])
	# 			update_pose = np.array([mu_x_tng1[0], mu_x_tng1[1], mu_x_tng1[2]])
	# 			ax.scatter(np.linspace(update_pose[0], root[0], 30), np.linspace(update_pose[1], root[1], 30), np.linspace(update_pose[2], root[2], 30), c='g', marker='.') # uncomment to not plot the stem of teh updated pose
	# 		loop = loop + 1
	# 	elif len(j) == 0 and len(goal_update)!=0:
	# 		if (loop % 2 == 0):
	# 			ax.scatter(mu_x_tng1[0], mu_x_tng1[1], mu_x_tng1[2], s = 100, c='r', marker='o')
	# 		else:
	# 			ax.scatter(mu_x_tng1[0], mu_x_tng1[1], mu_x_tng1[2], s = 100, c='g', marker='o')
	# 		##### STEMS DRAWINGS 
	# 		#ST_g1n
	# 		if (loop % 2 != 0):
	# 			print('loop is even')
	# 		else:
	# 			print('loop is odd')
	# 			ax.scatter(np.repeat(mu_x_tng1[0], 80), np.repeat(mu_x_tng1[1], 80), np.linspace(mu_x_tng1[2], mu_x_tng1[2]+0.2, 80), c='g', marker='.')
	# 		loop = loop + 1
	# 		ax.scatter(np.linspace(mu_x_tf[0], cluster_init[0], 80), np.linspace(mu_x_tf[1], cluster_init[1], 80), np.linspace(mu_x_tf[2], cluster_init[2]+0.2, 80), c='g', marker='.')
	# 	else:
	# 		# first is the goal (done out of loop)
	# 		if l == 0:
	# 			#ax.scatter(mu_x_tng1[0], mu_x_tng1[1], mu_x_tng1[2], s = 100, c='g', marker='o')
	# 			loop = loop + 1
	# 		else:
	# 			if (loop % 2 == 0):
	# 				ax.scatter(mu_x_tng1[0], mu_x_tng1[1], mu_x_tng1[2], s = 100, c='g', marker='o')
	# 			else:
	# 				ax.scatter(mu_x_tng1[0], mu_x_tng1[1], mu_x_tng1[2], s = 100, c='r', marker='o')
	# 			##### STEMS DRAWINGS
	# 			#ST_g1n
	# 			if (loop % 2 != 0):
	# 				print('loop is odd')
	# 				ax.scatter(np.repeat(mu_x_tng1[0], 80), np.repeat(mu_x_tng1[1], 80), np.linspace(mu_x_tng1[2], mu_x_tng1[2]+0.2, 80), c='g', marker='.')
	# 			else:
	# 				print('loop is even')
	# 				mu_x_tng1_prev = mu_x_tfn[l-1]
	# 				root = np.array([mu_x_tng1_prev[0], mu_x_tng1_prev[1], mu_x_tng1_prev[2]+0.2])
	# 				update_pose = np.array([mu_x_tng1[0], mu_x_tng1[1], mu_x_tng1[2]])
	# 				ax.scatter(np.linspace(update_pose[0], root[0], 80), np.linspace(update_pose[1], root[1], 80), np.linspace(update_pose[2], root[2], 80), c='g', marker='.')
	# 			loop = loop + 1
	# 		ax.scatter(np.linspace(mu_x_tf[0], mu_x_tfn[-1][0], 80), np.linspace(mu_x_tf[1], mu_x_tfn[-1][1], 80), np.linspace(mu_x_tf[2], mu_x_tfn[-1][2]+0.2, 80), c='g', marker='.')

	# # ST_g1
	# ax.scatter(traj_Xee_condt1g1, traj_Yee_condt1g1, traj_Zee_condt1g1, c='b', marker='.')
	# ax.scatter(np.repeat(cluster_init[0], 80), np.repeat(cluster_init[1], 80), np.linspace(cluster_init[2], cluster_init[2]+0.2, 80), c='g', marker='.')
	# ##
	# if len(goal_update)==0:
	# 	ax.scatter(mu_x_tf[0], mu_x_tf[1], mu_x_tf[2], s = 100, c='r', marker='o')
	# else:
	# 	ax.scatter(mu_x_tf[0], mu_x_tf[1], mu_x_tf[2], s = 100, c='g', marker='o')
	# ax.scatter(mu_x_t1g1[0], mu_x_t1g1[1], mu_x_t1g1[2], s = 100, c='c', marker='o')
	# ax.scatter(mu_x_t0[0], mu_x_t0[1], mu_x_t0[2], s = 100, c='y', marker='o')
	# ax.scatter(cluster_init[0], cluster_init[1], cluster_init[2]+0.016, s = 100, c='r', marker='o')  # we conditioned below th goal by 0.016 but we want to plot the goal, so we reput 0.016
	# ax.text(mu_x_t0[0]-0.015, mu_x_t0[1], mu_x_t0[2]+0.03, 'IC', style='italic', weight = 'bold')
	# ax.text(cluster_init[0]-0.015, cluster_init[1], cluster_init[2]+0.25, 'Goal', style='italic', weight = 'bold')
	# ax.set_xlabel('X')
	# ax.set_ylabel('Y')
	# ax.set_zlabel('Z')
	# #plt.title('Task space cond @ goal with pushing @ NN')
	# plt.show()


if __name__ == "__main__":
	#initialization = promp_pub()
	#while (initialization.q0.shape[0] == 0):
	#	rospy.loginfo('init state subscriber didnt return yet')
	#	time.sleep(1)
	intpromp_generator = promp_pub()
	data, time = intpromp_generator.load_demos()
	Q = data 
	bunch_xyz, bunch_col, bunch_orient, conf = cluster_request()
	IC0, cluster_init, cond_pts, min_wpts2, subset_Ncond, goal_update = franka_actions.PushOpt_planner(bunch_xyz, bunch_col, bunch_orient, conf)
	GoalCond_promp, Goalpromp_Sampled, time_normalised, nDof, mu_ang_euler_final, sig_euler, ttrajectories_learned = intpromp_generator.promp_generator(IC0, cluster_init)
	#plot_traj(ttrajectories_learned, cond_pts, IC0, cluster_init, cam = None)
	IntProMP_joint, IntProMP_task, cond_pts, tcond, x_cam, mean_jcondpts, cov_jcondpts = pushing_generator(GoalCond_promp, Goalpromp_Sampled, cond_pts, time_normalised, nDof, mu_ang_euler_final, sig_euler)
	plot_traj(IntProMP_task, cond_pts, IC0, cluster_init, cam =x_cam)
	#IntProMP_Opt = intpromp_generator.intpromp_opt(IntProMP_joint, cond_pts, tcond, mean_jcondpts, cov_jcondpts)
	#intpromp_generator.jointTrajectoryCommand(IntProMP_joint, t=20)
