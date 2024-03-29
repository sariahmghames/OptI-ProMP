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
from scipy.optimize import LinearConstraint, Bounds, NonlinearConstraint
from scipy.optimize import Bounds
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import basis as basis
import promps as promps
import phase as phase
import tf.transformations as tf_tran
import matplotlib.pyplot as plt
import franka_kinematics
from mpl_toolkits.mplot3d import Axes3D
import Franka_pushing_clusters as franka_decluster
import formulations as fm
import traj_opt    


franka_kin = franka_kinematics.FrankaKinematics()
franka_actions = franka_decluster.actions()

# This is the latest script in use to generate conditioned promp for franka, then the push_cost will create pushing actions
# the franka_intprompV2Opt.py has the pushing actions generated as IROS paper for Scara, pushing actions are not well generated as demos need to be collected 
# with franka for the specific picking task

class promp_pub(object):

	def __init__(self):
		#rospy.init_node('franka_intprompV2_publisher', anonymous=True)
		#self.rate = rospy.Rate(50)
		self.q0 = np.array([])
		self.q0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3]  # comment out wen running sim
		self.Tf = 0.3
		self.T0 = 0
		self.nDof = 7
		self.t1samples = 200
		self.t2samples = 200
		self.time_normalised1 = np.linspace(self.T0, self.Tf, self.t1samples)
		self.t_cam2 = self.time_normalised1[-1]-0.2
		self.init_cov = np.eye(len(self.q0)) * 0.000001
		self.Rgrip = 0.03
		self.k = 0.002
		self.Xi = 0.002
		self.pot_power = 2
		self.Q_star = 0.01 # change later
		self.constraints = {}
		self.goal = np.array([])
		self.traj_opt = traj_opt.trajectory_optimization(goal = self.goal,const =self.constraints, tcam2 = self.t_cam2, tsamples = self.t1samples, tf = self.Tf)

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
		basisGenerator1 = basis.NormalizedRBFBasisGenerator(phaseGenerator, numBasis=4, duration=self.Tf, basisBandWidthFactor=5, # check duration
		                                                   numBasisOutside=1)


	
		proMP1 = promps.ProMP(basisGenerator1, phaseGenerator, self.nDof)
		plotDof = 2

		################################################################
		# Conditioning in JointSpace

		desiredTheta = np.array([0.5, 0.7, 0.5, 0.2, 0.6, 0.8, 0.1])
		desiredVar = np.eye(len(desiredTheta)) * 0.0001
		meanTraj, covTraj = proMP1.getMeanAndCovarianceTrajectory(self.time_normalised1)
		newProMP1 = proMP1.jointSpaceConditioning(self.Tf, desiredTheta=desiredTheta, desiredVar=desiredVar)
		traj = proMP1.getTrajectorySamples(self.time_normalised1, 1)
		print('traj =', traj.shape)

		#plt.figure()
		#plt.plot(time_normalised, trajectories[:, plotDof, :])
		#plt.xlabel('time')
		#plt.title('Joint-Space conditioning for joint 2')

		proMP1.plotProMP(self.time_normalised1, [3, 4]) # refer to plotter.py, it plots means ans 2*std filled curves of newpromp, indices = [3, 4] refer to joints to plot


		##################################################
		# Conditioning in Task Space

		learnedProMP1 = promps.ProMP(basisGenerator1, phaseGenerator, self.nDof) # regression model initialization
		learner1 = promps.MAPWeightLearner(learnedProMP1) # weights learning model
		#print('q', Q[:][:120])
		learner1.learnFromData(self.Q, self.time) # get weights
		learned_promp1 = learnedProMP1.getTrajectorySamples(self.time_normalised1, 1)
		ttrajectories_learned = franka_kin.fwd_kin_trajectory(learned_promp1)
		mu_theta, sig_theta = learnedProMP1.getMeanAndCovarianceTrajectory(np.array([self.Tf])) # get mean cov of the learnt promp in joint space at time T = 1s
		#print('mu_theta=', np.squeeze(mu_theta))
		#print('sig_theta=', np.squeeze(sig_theta))
		sig_theta = np.squeeze(sig_theta)
		mu_x = goal # desired mean ee pose/position at T = 1s, old= [0.6, 0.5, 0.8]
		sig_x = np.eye(3) * 0.0000001 # desired cov of ee pose/position at T = 1s
		q_home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3]
		T_desired, tmp = franka_kin.fwd_kin(q_home)
		mu_ang_euler_des = tf_tran.euler_from_matrix(T_desired, 'szyz')
		sig_euler = np.eye(3) * 0.0001
		print('starting IK')
		post_mean_Ash, post_cov = franka_kin.inv_kin_ash_pose(np.squeeze(mu_theta), sig_theta, mu_x, sig_x, mu_ang_euler_des, sig_euler) # output is in joint space, desired mean and cov
		#print('post_mean =',post_mean_Ash.shape) 7x1
		#print('post_cov =',post_cov.shape) 7x7
		print('finishing IK')
		newProMP0 = learnedProMP1.jointSpaceConditioning(self.T0, desiredTheta= self.q0, desiredVar=self.init_cov)
		newProMP1 = newProMP0.jointSpaceConditioning(self.Tf, desiredTheta= post_mean_Ash, desiredVar=post_cov)
		trajectories_task1_conditioned = newProMP1.getTrajectorySamples(self.time_normalised1, 1)
		ttrajectories_task_conditioned = franka_kin.fwd_kin_trajectory(trajectories_task1_conditioned)

		# Get orientation of EE at T = 1s from demonstrations
		q_final = mu_theta
		q_final = np.squeeze(q_final)
		print('q_final=', q_final)
		T_final, tmp_final = franka_kin.fwd_kin(q_final)
		mu_ang_euler_final = tf_tran.euler_from_matrix(T_final, 'szyz')

		return newProMP1, ttrajectories_task_conditioned, mu_ang_euler_final, sig_euler, ttrajectories_learned


	def pushing_generator(self, GoalCond_promp, Goalpromp_Sampled, cond_pts, mu_ang_euler_final, sig_euler):
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

		t0 = self.time_normalised1[0]
		tf = self.time_normalised1[-1]
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
		mu_theta1, sig_theta1 = GoalCond_promp.getMeanAndCovarianceTrajectory(np.array([self.t_cam2])) # get mean cov of the learnt promp in joint space at time T = 1s
		sig_theta1 = np.squeeze(sig_theta1)
		mu_x_t1g1 = [cluster_init[0], cluster_init[1], (minZ-0.1)]  # desired mean ee pose/position at T = 1s
		sig_x_t1g1 = np.eye(3) * 0.0000002 # desired cov of ee pose/position at T = 1s
		post_mean_push, post_cov_push = franka_kin.inv_kin_ash_pose(np.squeeze(mu_theta1), sig_theta1, mu_x_t1g1, sig_x_t1g1, mu_ang_euler_final, sig_euler) # output is in joint space, desired mean and cov
		print('finishing IK')
		newProMP1 = GoalCond_promp.jointSpaceConditioning(self.t_cam2, desiredTheta=post_mean_push, desiredVar=post_cov_push)
		trajectories_task1_conditioned = newProMP1.getTrajectorySamples(self.time_normalised1, 1)

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

			
		t_discrete = np.linspace(self.t_cam2, tf, len(mu_x_tfn)+1,endpoint=False) # len(mu_x_tfn)
		print('t_discrete=', t_discrete)
		print('mu_x_tfn=', len(mu_x_tfn))
		t_effec = []
		mean_push = []
		cov_push = []

		traj_cond = []
		traj_cond.append(newProMP1)

		for k in range(1,len(mu_x_tfn)+1): # range(0,len(mu_x_tfn))
			## Nonlinear scale of t_discrete : exponential one
			tn = t_discrete[k]
			if (k % 2) != 0:
				tn = t_discrete[k] - t_discrete[k] * 0.0 # Tune those for trajectory variation minimization, + t_discrete[k+1] * 0.1 only add it in simulation to increase time to reach 1st cond point
			elif k==(len(mu_x_tfn)):
				tn = t_discrete[k] - t_discrete[k] * 0.1 # not much diff if 0.0
			else:
				tn = t_discrete[k] - t_discrete[k] * 0.1 # not much diff if 0.0
			t_effec.append(tn)
			print('tn=', tn)
			mu_thetan, sig_thetan = traj_cond[k-1].getMeanAndCovarianceTrajectory(np.array([tn])) # get mean cov of the learnt promp in joint space at time T = 1s
			sig_thetan = np.squeeze(sig_thetan)
			mu_x_tn = mu_x_tfn[k-1] 
			#print('mu_x=',mu_x_tng1)
			sig_x_tn = np.eye(3) * 0.0000002 # desired cov of ee pose
			post_mean_push, post_cov_push = franka_kin.inv_kin_ash_pose(np.squeeze(mu_thetan), sig_thetan, mu_x_tn, sig_x_tn, mu_ang_euler_final, sig_euler) # output is in joint space, desired mean and cov
			mean_push.append(post_mean_push)
			cov_push.append(post_cov_push)
			newProMPn = traj_cond[k-1].jointSpaceConditioning(tn, desiredTheta=post_mean_push, desiredVar=post_cov_push)
			trajectories_taskn_conditioned = newProMPn.getTrajectorySamples(self.time_normalised1, 1)

			traj_cond.append(newProMPn)

		## Save data for testing
	  	jtrajectories_pushed = trajectories_taskn_conditioned
		print('traj pushed jt=', jtrajectories_pushed.shape) 
		ttrajectories_pushed = franka_kin.fwd_kin_trajectory(np.squeeze(jtrajectories_pushed))
		print('traj pushed task=', ttrajectories_pushed.shape) 

		with open('jintprompV2_Pushing.npz', 'w') as f:
			np.save(f, jtrajectories_pushed)
		return jtrajectories_pushed, ttrajectories_pushed, cond_pts, t_effec, mu_x_t1g1, mean_push, cov_push



	def optCallback(self, xk, state):   # xk is joints traj, here we calculate total costs based on traj of joints
        # xk = xk.reshape(len(xk) / 7, 7)
		print("xk size: {}\n".format(len(xk)))
		costs = self.traj_opt.calculate_total_cost(xk) # xk as traj wll be nxlen(body pts)
		self.traj_opt.optCurve.append(xk) # xk is the result of each optimization problem iteration
		print("Iteration {}: {}\n".format(len(self.traj_opt.optCurve), costs))  # first {} refers to first element of format() and second {} refers to second element


	def linear_constraints_sariah(self, trajectoryFlat, trajectory, mean, cov):
		# Assign a timestamp for each condpt
		lc= []
		lb = []
		ub = []
		connections = []
		for elem in range(len(self.constraints['condpts'])):
			connection = np.linspace(self.constraints['condpts'][elem], self.constraints['condpts'][elem]+self.constraints['lstems'][elem]*np.array([0,0,np.cos(self.constraints['incl'][elem][0])]), 80)
			for ele in connection:
				connections.append(ele)
		print('trajflat shape for LC=', trajectoryFlat.shape)
		tcam_samp = str(np.round(self.t_cam2*(self.t1samples)/self.Tf,0))
		tcam_samp = tcam_samp.rstrip('0').rstrip('.') if '.' in tcam_samp else tcam_samp
		tcam_samp = int(tcam_samp)
		rt, ct, dt = trajectory.shape
		print('tcam=',tcam_samp)
		print('traj shape= {} {} {}'.format(rt, ct, dt))
		traj_sub = trajectory[tcam_samp:, :, :]
		rts, cts, dts = traj_sub.shape
		traj_base = np.transpose(trajectoryFlat.reshape((7, len(trajectoryFlat) / 7))) # len(traj) is along the highest btw rows and col, after transpose it becomes nx7

		if self.constraints:
			print('Constraints are available')
			for dem in range(dts):
				for spls in range(len(traj_sub)):
					T, Tj = franka_kin.fwd_kin(traj_sub[spls,:,dem])
					pt1 = T[0:3,3]
					#print('pt1=', pt1)
					pt2 = self.constraints['goal']
            		#dist_norm = np.norm(dist)
					dist = pt1 - pt2
					d_proj = np.absolute(self.traj_opt.table_unit.dot(dist))
					p1 = np.linspace(pt1, pt2, 80, endpoint=True)
					#print('set p1=', p1)
					#print('set connections=', connections)
					inter_pts = p1 in np.asarray(connections)
					stem_inter = np.any(p1 in np.asarray(connections))
					if stem_inter == 'False':
						stem_inter = 0
					else:
						stem_inter = 1
					#print('bool=',stem_inter)
					joint0_var = traj_sub[spls,0,dem]
					joint1_var = traj_sub[spls,1,dem]
					joint2_var = traj_sub[spls,2,dem]
					joint3_var = traj_sub[spls,3,dem]
					joint4_var = traj_sub[spls,4,dem]
					joint5_var = traj_sub[spls,5,dem]
					joint6_var = traj_sub[spls,6,dem]
					joint_var = [joint0_var, joint1_var, joint2_var, joint3_var, joint4_var, joint5_var, joint6_var]
					lc.extend([d_proj, stem_inter])
					lb.extend([self.Rgrip, 0])
					ub.extend([np.inf, 0])
					bounds = Bounds([-80*(np.pi)/180,-80*(np.pi)/180,-80*(np.pi)/180,-80*(np.pi)/180,-80*(np.pi)/180,-80*(np.pi)/180,-80*(np.pi)/180], [80*(np.pi)/180, 80*(np.pi)/180, 80*(np.pi)/180, 80*(np.pi)/180, 80*(np.pi)/180, 80*(np.pi)/180, 80*(np.pi)/180])
					#print('finish appending')

		print('lc shape=', len(lc))
		print('lb shape=', len(lb))
		print('ub shape=', len(ub))
		print('bounds shape=', bounds)
		
		return LinearConstraint(lc, lb, ub), bounds


	def nonlinear_constraints_sariah_old(self, theta, trajectory):
		# Assign a timestamp for each condpt
		#lc= []
		#lb = []
		#ub = []
		connections = []
		for elem in range(len(self.constraints['condpts'])):
			connection = np.linspace(self.constraints['condpts'][elem], self.constraints['condpts'][elem]+self.constraints['lstems'][elem]*np.array([0,0,np.cos(self.constraints['incl'][elem][0])]), 80)
			for ele in connection:
				connections.append(ele)
		
		#tcam_samp = str(np.round(self.t_cam2*(self.t1samples)/self.Tf,0))
		#tcam_samp = tcam_samp.rstrip('0').rstrip('.') if '.' in tcam_samp else tcam_samp
		#tcam_samp = int(tcam_samp)
		#rt, ct, dt = trajectory.shape
		#print('tcam=',tcam_samp)
		#print('traj shape= {} {} {}'.format(rt, ct, dt))
		#traj_sub = trajectory[tcam_samp:, :, :]
		#rts, cts, dts = traj_sub.shape
		#traj_base = np.transpose(trajectoryFlat.reshape((7, len(trajectoryFlat) / 7))) # len(traj) is along the highest btw rows and col, after transpose it becomes nx7

		#if self.constraints:
			#print('Constraints are available')
			#for dem in range(dt):
				#for spls in range(rt):
					#joint_values = np.array([trajflat[spls*franka_kin.numJoints-(franka_kin.numJoints-1)*dem], trajflat[spls*franka_kin.numJoints-(franka_kin.numJoints-2)*dem], trajflat[spls*franka_kin.numJoints-(franka_kin.numJoints-3)*dem], trajflat[spls*franka_kin.numJoints-(franka_kin.numJoints-4)*dem], trajflat[spls*franka_kin.numJoints-(franka_kin.numJoints-5)*dem], trajflat[spls*(franka_kin.numJoints-(franka_kin.numJoints-6))*dem], trajflat[spls*(franka_kin.numJoints-(franka_kin.numJoints-7))*dem]])
					#print('about to start fwd')
		T, Tj = franka_kin.fwd_kin(theta)
		print('about to finish fwd')
		pt1 = T[0:3,3]
		#print('pt1=', pt1)
		pt2 = self.constraints['goal']
		#dist_norm = np.norm(dist)
		dist = pt1 - pt2
		d_proj = np.absolute(self.traj_opt.table_unit.dot(dist))
		p1 = np.linspace(pt1, pt2, 80, endpoint=True)
		#print('set p1=', p1)
		#print('set connections=', connections)
		inter_pts = p1 in np.asarray(connections)
		stem_inter = np.any(p1 in np.asarray(connections))
		if stem_inter == 'False':
			stem_inter = 0
		else:
			stem_inter = 1
		#print('bool=',stem_inter)
		joint0_var = trajectory[spls,0,dem]
		joint1_var = trajectory[spls,1,dem]
		joint2_var = trajectory[spls,2,dem]
		joint3_var = trajectory[spls,3,dem]
		joint4_var = trajectory[spls,4,dem]
		joint5_var = trajectory[spls,5,dem]
		joint6_var = trajectory[spls,6,dem]
		joint_var = [joint0_var, joint1_var, joint2_var, joint3_var, joint4_var, joint5_var, joint6_var]
		print('d_proj=', d_proj)
		print('stem_inter=', stem_inter)
		#lc.extend([d_proj, stem_inter])
		#lb.extend([self.Rgrip, 0])
		#ub.extend([np.inf, 0])
		#bounds = Bounds([-80*(np.pi)/180,-80*(np.pi)/180,-80*(np.pi)/180,-80*(np.pi)/180,-80*(np.pi)/180,-80*(np.pi)/180,-80*(np.pi)/180], [80*(np.pi)/180, 80*(np.pi)/180, 80*(np.pi)/180, 80*(np.pi)/180, 80*(np.pi)/180, 80*(np.pi)/180, 80*(np.pi)/180])
		#print('finish appending')											

	
		#nLC = nonlinearConstraint(np.asarray(lc), np.asarray(lb), np.asarray(ub))

		
		return d_proj, stem_inter



	def nonlinear_constraints_sariah(self, theta):
	
		connections = []
		for elem in range(len(self.constraints['condpts'])):
			connection = np.linspace(self.constraints['condpts'][elem], self.constraints['condpts'][elem]+self.constraints['lstems'][elem]*np.array([0,0,np.cos(self.constraints['incl'][elem][0])]), 80)
			for ele in connection:
				connections.append(ele)
		c1 = []
		c2 = []		

		init_traj = np.transpose(theta.reshape(7,len(theta)/7))
		r, c = init_traj.shape

		for spls in range(len(init_traj)):
			q = init_traj[spls,:]
			indices = np.where(np.in1d(theta, q))[0]
			
			T, Tj = franka_kin.fwd_kin([theta[indices[0]], theta[indices[1]], theta[indices[2]], theta[indices[3]], theta[indices[4]], theta[indices[5]], theta[indices[6]]])
			pt1 = T[0:3,3]
			#print('pt1=', pt1)
			pt2 = self.constraints['goal']
			#dist_norm = np.norm(dist)
			dist = pt1 - pt2
			d_proj = np.absolute(self.traj_opt.table_unit.dot(dist))
			p1 = np.linspace(pt1, pt2, 80, endpoint=True)

			inter_pts = p1 in np.asarray(connections)
			stem_inter = np.any(p1 in np.asarray(connections))
			if stem_inter == 'False':
				stem_inter = 0
			else:
				stem_inter = 1
			#print('bool=',stem_inter)
			joint0_var = init_traj[spls,0]
			joint1_var = init_traj[spls,1]
			joint2_var = init_traj[spls,2]
			joint3_var = init_traj[spls,3]
			joint4_var = init_traj[spls,4]
			joint5_var = init_traj[spls,5]
			joint6_var = init_traj[spls,6]
			joint_var = [joint0_var, joint1_var, joint2_var, joint3_var, joint4_var, joint5_var, joint6_var]
			#print('d_proj=', d_proj)
			#print('stem_inter=', stem_inter)
			#print('spls=', spls)

			c1.append(d_proj)
			c2.append(stem_inter)
		
		#print('c1=', len(c1))
		#print('c2=', len(c2))
		constr = np.concatenate((c1, c2), axis=0)
		#constr = [np.asarray(c1), np.asarray(c2)]
		#constr = np.asarray(constr).reshape(-1)
		return constr



	def intprompV2_opt_sariah(self, condpromp, goal, cond_pts, tcond, inclination, Lstems, mean, cov):
		self.constraints = {'goal': goal, 'condpts':cond_pts,'tcond':tcond, 'incl': inclination, 'lstems':Lstems }
		self.goal = goal
		print('goal=', self.goal)
		self.traj_opt = traj_opt.trajectory_optimization(goal = self.goal,const =self.constraints, tcam2 = self.t_cam2, tsamples = self.t1samples, tf = self.Tf)

		init_traj = condpromp # 200x7x1
		print('init traj=',init_traj[0,:,:])
		#init_traj = np.squeeze(init_traj) # 200x7
		b, r , c= init_traj.shape  # b: blocks = 100 (time), r = rows(7, dof) and c = columns (20, samples from the Gaussian distribution at each dof at an instant of time)
		print('traj dim is: {}, {}, {}'.format(b, r, c))
		#print('ProMP traj is: {}'.format(init_traj))
	 
		itraj_size = len(init_traj) 
		print('init traj size: {}'.format(itraj_size))

		initial_joint_values = init_traj[0,:,:]
		desired_joint_values = init_traj[(itraj_size - 1),:,:]
		trajectoryFlat = init_traj.T.reshape(-1) # by transpose: we got 20x7x100, by reshape(-1) we merge the elements of the original array in one line
		print('trajectoryflat = ',trajectoryFlat[200])
		bf = trajectoryFlat.shape
		print('probabilistic_trajflat dim is: {}'.format(bf))

		traj_mean = np.mean(init_traj, axis=2)
		bm, rm = traj_mean.shape
		print('mean proba traj dim is: {}, {}'.format(bm, rm)) 
		obst_cost_grad_analytic = np.reshape(self.traj_opt.calculate_obstacle_cost_gradient(trajectoryFlat), -1)
		#print('obst_cost_grad_analytic {} \n'.format(obst_cost_grad_analytic))  # gradient cost of initialized traj 

		################### With Gradient ############################
	    # minimize total cost
	    # scipy.minimize(fun, x0, args=(), method='BFGS', jac=None, tol=None, callback=None, options={'gtol': 1e-05, 'norm': inf, 'eps': 1.4901161193847656e-08, 'maxiter': None, 'disp': False, 'return_all': False})
	    # BFGS solver uses a limited computer memory and it is a popular algorithm for parameter estimation in machine learning.
	    # disp : Set to True to print convergence messages.
	    # mixter :max nb of iterations
	    # gtol : Gradient norm must be less than gtol before successful termination.


		c1_lb = []
		c1_ub = []
		c2_lb = []
		c2_ub = []
		bounds_up = []
		bounds_low = []
		connections = []

		if self.constraints:
			print('Constraints are available')
			for dem in range(c):
				for spls in range(b):
					c1_lb.append(self.Rgrip)
					c2_lb.append(0)
					c1_ub.append(np.inf)
					c2_ub.append(0)
					bounds_low.append([-80*(np.pi)/180,-80*(np.pi)/180,-80*(np.pi)/180,-80*(np.pi)/180,-80*(np.pi)/180,-80*(np.pi)/180,-80*(np.pi)/180])
					bounds_up.append([80*(np.pi)/180, 80*(np.pi)/180, 80*(np.pi)/180, 80*(np.pi)/180, 80*(np.pi)/180, 80*(np.pi)/180, 80*(np.pi)/180])
			
			#lb = [np.asarray(c1_lb), np.asarray(c2_lb)]
			#ub = [np.asarray(c1_ub), np.asarray(c2_ub)]
			#lb = np.asarray(lb).reshape(-1)
			#ub = np.asarray(ub).reshape(-1)
			lb = np.concatenate((c1_lb, c2_lb), axis=0)
			ub = np.concatenate((c1_ub, c2_ub), axis=0)

			print('ub shape=', np.asarray(ub).shape)
			print('lb shape=', np.asarray(lb).shape)
			up_bounds = np.asarray(bounds_up).T.reshape(-1)
			low_bounds = np.asarray(bounds_low).T.reshape(-1)
			
			bounds = Bounds(low_bounds, up_bounds)
			print('finish appending')

			constr = lambda theta: self.nonlinear_constraints_sariah(theta)											
	
			nLC = NonlinearConstraint(constr, lb, ub)
			print('finished formulating the NL constraints')
			#print('func=', init_traj[spls,:,dem].T.reshape(-1))
			# Ok works: optimized_trajectory = opt.minimize(self.traj_opt.calculate_total_cost, trajectoryFlat, method='trust-constr', jac=self.traj_opt.cost_gradient_analytic, bounds = bounds, callback=self.optCallback, options={'maxiter': 15, 'disp': True})
			optimized_trajectory = opt.minimize(self.traj_opt.calculate_total_cost, trajectoryFlat, method='trust-constr', jac=self.traj_opt.cost_gradient_analytic, bounds = bounds, constraints = nLC, callback=self.optCallback, options={'maxiter': 3, 'disp': True})
			print('finished inner optimization')

			#init_traj[spls,:,dem].T.reshape(-1)

			#optimized_trajectory = opt.minimize(self.traj_opt.calculate_total_cost, trajectoryFlat, method='BFGS', jac=self.traj_opt.cost_gradient_analytic, options={'maxiter': 15, 'disp': True}, callback=self.traj_opt.optim_callback)
		#optimized_trajectory = opt.minimize(self.traj_opt.calculate_total_cost, trajectoryFlat, method='trust-constr', jac=self.traj_opt.cost_gradient_analytic, bounds = bounds, constraints = nLC, callback=self.optCallback, options={'maxiter': 15, 'disp': True})
		print('passed the optimization, opt traj=', optimized_trajectory)
		optimized_trajectory = np.transpose(optimized_trajectory.x.reshape((7, len(optimized_trajectory.x) / 7)))
		#optimized_trajectory = np.insert(optimized_trajectory, 0, initial_joint_values, axis=0)
		#optimized_trajectory = np.insert(optimized_trajectory, len(optimized_trajectory), desired_joint_values, axis=0)
		optimized_trajectory[0] = np.squeeze(initial_joint_values)
		#optimized_trajectory[-1,:] = desired_joint_values
		print('result of optimized trajectory {}\n'.format(optimized_trajectory))
		self.traj_opt.animation(optimized_trajectory, init_traj)

		print('Finished plotting the optimized trajectory with Gradient passed to the minimization Algo')

		print('finished')	



	def switch_control_old(self, traj_opt, grid_init, neigh, incl):
		hybrid_traj = []
		X_old = np.zeros(3,)
		switch_traj = np.array([])
		switch_loop = 0
		for col in range(0,traj_opt.shape[1]):
			T, Tj = franka_kin.fwd_kin(traj_opt[:,col])[0:2]
			Xee = T[0:3,3]
			Up, fp = self.force_field(grid_init, Xee, neigh, incl)
			X_old = Xee
			closest_opt, ind_opt= find_nearest(grid_init[0,:], Xee[0])  
			if np.abs(grid_init[2,ind_opt]-Xee[2]) <= 0.15:
				if Fp[ind_opt]!= 0:
					np.loginfo('switch the control to APF')
					hybrid_traj.append(traj_opt[:,:col])
					# walk left or right ?
					if Fp[traj_opt+1] > Fp[traj_opt-1]:
						# walk right
						i_null = np.where(Fp[ind_opt:]==0)
						i_null = i_null[0]
						for w in range(traj_opt+1, i_null-1):
							switchX_new = X_old + Fp[w]/self.k
							#T_desired = ####
							switch_traj[:,switch_loop] = franka_kin.inv_kin2(traj_opt[:,col], T_desired)
							switch_loop = switch_loop + 1
					else:
						# walk left
						i_null = np.where(Fp[:ind_opt]==0)
						i_null = i_null[0]
						for w in range(i_null+1, traj_opt-1):
							switchX_new = X_old - Fp[w]/self.k
							#T_desired = ####
							switch_traj[:,switch_loop] = franka_kin.inv_kin2(traj_opt[:,col], T_desired)
							switch_loop = switch_loop + 1
					switch_loop = 0
					hybrid_traj.append(switch_traj)
		L_hybrid = len(hybrid_traj)
		hybrid_traj = np.asarray(hybrid_traj)
		hybrid_traj.reshape(1,L_hybrid)
		return hybrid_traj


	def switch_control(self, traj_opt, grid_init, neigh, incl):
		hybrid_traj = []
		X_old = np.zeros(3,)
		switch_traj = np.array([])
		switch_loop = 0
		for col in range(0,traj_opt.shape[1]):
			T, Tj = franka_kin.fwd_kin(traj_opt[:,col])
			Xee = T[0:3,3]
			Up, Fp = self.force_field(grid_init, Xee, neigh, incl)
			X_old = Xee
			T_old = T
			closest_opt, ind_opt= find_nearest(grid_init[0,:], Xee[0])  
			if np.abs(grid_init[2,ind_opt]-Xee[2]) <= 0.15:
				if Fp!= 0:
					np.loginfo('switch the control to APF')
					hybrid_traj.append(traj_opt[:,:col])
					switchX_new = X_old + Fp/self.k
					x_dis = np.linspace(X_old, switchX_new, 3, endpoint=True)
					R_desired = T_old[0:3, 0:3]
					for w in range(1, len(x_dis)):
						t_desired = tf_tran.traslation_matrix(x_dis[w][0], x_dis[w][1], x_dis[w][2])
						T_desired = tf_tran.concatenate_matrices(R_desired, t_desired) 
						switch_traj[:,switch_loop] = franka_kin.inv_kin2(traj_opt[:,col], T_desired)
						switch_loop = switch_loop + 1

					switch_loop = 0
					hybrid_traj.append(switch_traj)
		L_hybrid = len(hybrid_traj)
		hybrid_traj = np.asarray(hybrid_traj)
		hybrid_traj.reshape(1,L_hybrid)
		return hybrid_traj


	def grid_eval(self, goal, neigh, incl):
		x_grid = np.linspace(goal[0]-0.1, goal[0]+0.1, 10)
		y_grid = np.linspace(goal[1]-0.1, goal[1]+0.1, 10)
		z_grid = np.linspace(goal[2]-0.1, goal[2]+0.1, 10)

		#x_grid, y_grid, z_grid = np.meshgrid(x_, y_, z_, indexing='xy')
		grid_eval = np.vstack(np.meshgrid(x_grid,y_grid,z_grid)).reshape(3,-1).T

		return grid_eval

	def force_field_old(self, grid_eval, Xee, neigh, incl):
		
		Up = np.zeros((len(grid_eval), len(grid_eval), len(grid_eval))) # potential field
		grad_Up = np.zeros((len(grid_eval), len(grid_eval), len(grid_eval))) # gradient of potential field
		Fp = np.zeros((len(grid_eval), len(grid_eval), len(grid_eval)))

		for el in range(len(neigh)):
			closest_pt, ind = self.find_nearest(grid_eval,neigh[el])
			d = Xee - neigh[el]  # check this
			#del_D_delX = 
			if incl[el][0] > 0:
				Up[ind[0]+1:, ind[1], ind[2]] = 0.5 * self.Xi * (d^2) # att  .. ceck index foro 3d array
				Up[ind] = 0.5 * self.Xi * (1/d - 1/self.Q_star)^2 # repulsive
				Up[:ind[0]-1, ind[1], ind[2]] = 0.5 * self.Xi * (1/d - 1/self.Q_star)^2 # repulsive
				grad_Up[ind[0]+1:,ind[1], ind[2]] = self.Xi * (Xee)
				grad_Up[ind] = self.Xi * (1/self.Q_star - 1/d) * (1/(d^2)) * del_D_delX
				grad_Up[:ind[0]-1, ind[1], ind[2]] = self.Xi * (1/self.Q_star - 1/d) * (1/(d^2)) * del_D_delX
			else:
				Up[:ind[0]-1, ind[1], ind[2]] = 0.5 * self.Xi * (d^2) # att
				Up[ind] = 0.5 * self.Xi * (1/d - 1/self.Q_star)^2 # repulsive
				Up[ind[0]+1:, ind[1], ind[2]] = 0.5 * self.Xi * (1/d - 1/self.Q_star)^2 # repulsive
				grad_Up[:ind[0]-1, ind[1], ind[2]] = self.Xi * (Xee)
				grad_Up[ind] = self.Xi * (1/self.Q_star - 1/d) * (1/(d^2)) * del_D_delX
				grad_Up[ind[0]+1:, ind[1], ind[2]] = self.Xi * (1/self.Q_star - 1/d) * (1/(d^2)) * del_D_delX

			Fp[ind] = -grad_Up[ind]
			Fp[:ind-1] = -grad_Up[:ind-1] # correct index
			Fp[ind+1:] = -grad_Up[ind+1:]

		return grid_eval


	def force_field(self, grid_eval, Xee, neigh, incl):
		
		Up = np.zeros((len(grid_eval), len(grid_eval), len(grid_eval))) # potential field
		grad_Up = np.zeros((len(grid_eval), len(grid_eval), len(grid_eval))) # gradient of potential field
		Fp = np.zeros((len(grid_eval), len(grid_eval), len(grid_eval)))

		for el in range(len(neigh)):
			closest_pt, ind = self.find_nearest(grid_eval,neigh[el])
			d = Xee - neigh[el]  # check this
			#del_D_delX = 
			if incl[el][0] > 0 and (self.traj_opt.table_unit.dot(d) < 0):
				Up = 0
				grad_Up = 0

			elif incl[el][0] > 0 and (self.traj_opt.table_unit.dot(d) > 0):

				Up = self.pot_power * 0.5 * self.Xi * (np.linalg.norm(d))^2 # att 
				grad_Up = self.pot_power * self.Xi * np.linalg.norm(d) * (1/2*np.ling.norm(d)) * 2 * d
				grad_Up = self.pot_power * self.Xi *  d


			elif incl[el][0] < 0 and (self.traj_opt.table_unit.dot(d) > 0):
				Up = 0
				grad_Up = 0
			else:

				Up = self.pot_power * 0.5 * self.Xi * (np.linalg.norm(d)^2) # att 
				grad_Up = self.pot_power * self.Xi *  d


			Fp = -grad_Up

		return Up, Fp

	def find_nearest(self,array, value):
	    array = np.asarray(array)
	    idx = (np.abs(array - value)).argmin()
	    return array[idx], idx


	def plot_traj(self, traj, cond_pts, x0, xf, sph, lstems, cam =None):

		fig = plt.figure()
		ax = fig.add_subplot(111, projection='3d')
		ax.scatter(traj[:,0], traj[:,1], traj[:,2], c='b', marker='.')
		ax.scatter(x0[0], x0[1], x0[2], s = 100, c='y', marker='o')
		ax.scatter(xf[0], xf[1], xf[2], s = 100, c='r', marker='o')
		if cam != None:
			ax.scatter(cam[0], cam[1], cam[2], s = 100, c='c', marker='o')
		#counting = 0
		for Ip in range(len(cond_pts)):
			#if counting == 0:
			pt = cond_pts[Ip]
			ax.scatter(pt[0], pt[1], pt[2], s = 100, c='g', marker='o')
			X, Y, Z = fm.plot_3dseg(pt, sph[Ip], lstems[Ip])
			ax.plot3D(X, Y, Z, 'green')
		Xg, Yg, Zg = fm.plot_3dseg(xf, [0,0], lstems[-1])
		ax.plot3D(Xg, Yg, Zg, 'green')
		for obs in self.traj_opt.object_list:
			x, y, z, alpha = fm.plot_sphere(obs[0:3], obs[3:])
			sphere = ax.plot_surface(x, y, z, color='b', alpha=alpha)
	        plt.pause(.001)

		ax.set_xlabel('X')
		ax.set_ylabel('Y')
		ax.set_zlabel('Z')
		#plt.title('ProMP Conditioning at selected unripe pushable obstacles:\nCase of fruits')
		plt.show()


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
	goal8_n1 = np.array([0.1-0.28*np.sin((np.pi)/9), X_goal8[1], 0.46-0.28*np.cos((np.pi)/6)]) 
	goal8_n2 = np.array([-0.09+0.2*np.sin((np.pi)/7), X_goal8[1], 0.46-0.2*np.cos((np.pi)/8)])  

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


	g7_n1_so = (np.pi)/6 

	g8_n1_so = (np.pi)/20 
	g8_n2_so = -(np.pi)/20

	g9_n1_so = (np.pi)/8
	g9_n2_so = 0



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
		spherical_angles = [[g7_n1_so, 0]]
		Lstems = [0.23, 0.13]
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
		spherical_angles = [[g8_n1_so, 0], [g8_n2_so, 0]]
		Lstems = [0.28, 0.2, 0.13]
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
		spherical_angles = [[g9_n1_so,0], [g9_n2_so,0]] 
		Lstems = [0.28, 0.13, 0.2]
	else:
		print('configuration requested doesnt exist')

	return bunch_xyz, bunch_col, bunch_orient, spherical_angles, Lstems, conf



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
	intprompV2_generator = promp_pub()
	data, time = intprompV2_generator.load_demos()
	Q = data 
	bunch_xyz, bunch_col, bunch_orient, spherical, Lstems, conf = cluster_request()
	IC0, cluster_init, cond_pts, min_wpts2, subset_Ncond, goal_update = franka_actions.PushOpt_planner(bunch_xyz, bunch_col, bunch_orient, conf)
	GoalCond_promp, Goalpromp_Sampled, mu_ang_euler_final, sig_euler, ttrajectories_learned = intprompV2_generator.promp_generator(IC0, cluster_init)
	#plot_traj(ttrajectories_learned, cond_pts, IC0, cluster_init, cam = None)
	intprompV2_joint, intprompV2_task, cond_pts, tcond, x_cam, mean_jcondpts, cov_jcondpts = intprompV2_generator.pushing_generator(GoalCond_promp, Goalpromp_Sampled, min_wpts2, mu_ang_euler_final, sig_euler)
	intprompV2_generator.plot_traj(intprompV2_task, cond_pts, IC0, cluster_init, spherical, Lstems, cam =x_cam)
	intprompV2_Opt = intprompV2_generator.intprompV2_opt_sariah(intprompV2_joint, cluster_init, cond_pts, tcond, spherical, Lstems, mean_jcondpts, cov_jcondpts)
	#grid = intprompV2_generator.grid_eval(cluster_init, cond_pts,spherical)
	#hybrid_control = intprompV2_generator.switch_control(intprompV2_joint, grid, cond_pts, spherical)
	#intprompV2_generator.jointTrajectoryCommand(intprompV2_joint, t=20)
