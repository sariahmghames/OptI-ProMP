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
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Float64, Float64MultiArray
import tf.transformations as tf_tran
import matplotlib.pyplot as plt
import franka_kinematics
from mpl_toolkits.mplot3d import Axes3D
import formulations as fm
import traj_opt    
from matplotlib import animation
from matplotlib.animation import FuncAnimation


franka_kin = franka_kinematics.FrankaKinematics()


class promp_pub(object):

	def __init__(self):
		rospy.init_node('franka_prompOpt_publisher', anonymous=True)
		self.rate = rospy.Rate(10)
		self.q0 = np.array([])
		self.q0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # comment out wen running sim
		self.qf = [0.0, (-70*np.pi)/180, 0.0, (160*np.pi)/180, 0.0, (np.pi)/2, 0.0]   # works well for final config desired 
		self.qf_frwd = [0.0, (-70*np.pi)/180, 0.0, (160*np.pi)/180, 0.0, (-np.pi)/2, 0.0] 
		self.Tf = 0.5 # originally set to 0.3 --> chnage with task and with position of robot wrt cluster
		self.T0 = 0
		self.nDof = 7
		self.init_cov = np.eye(len(self.q0)) * 0.000001
		self.Rgrip = 0.1
		self.Rcluster = 0.1
		self.stem_ndiscrete = 100
		self.constraints = {}
		self.goal = np.array([-0.04998666399793127, 0.10000034078139142, 1.059910236708741]) # init to straw81
		self.neighbours = []
		self.connections = []
		self.object_push_list = []
		self.cost = []
		self.iter = []	
		self.ninter0 = 0
		self.ninter1 = 0
		#self.pubTrajD = rospy.Publisher('/effort_joint_controller/command', Float64MultiArray, queue_size=10)
		self.pubTrajD = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
		#self.subJState  = rospy.Subscriber("/joint_states", JointState, callback=self.RobotStateCallback)
		#self.subSPose  = rospy.Subscriber("/straws_states", PoseArray, callback=self.ObjCallback)
		self.Reset = False
		self.sat = False


	def RobotStateCallback(self,msg):
		name = msg.name
		print('msg state=', msg)
		self.q0 = msg.position[2:]
		print('q0=', self.q0)
		self.qdot0 = msg.velocity[2:]
		if (len(self.q0) != 0):
			self.subJState.unregister()



	def ObjCallback(self,msg):
		cluster = msg.poses
		cluster_frame = msg.header.frame_id
		self.goal = np.array([cluster[0].position.x, cluster[1].position.y, cluster[2].position.z]) 
		self.neighbours = [np.array([cluster[1].position.x, cluster[1].position.y, cluster[1].position.z]), np.array([cluster[2].position.x, cluster[2].position.y, cluster[2].position.z])]
		print('neighbours=',self.neighbours)

		if ((cluster_frame) and len(self.neighbours) != 0):
			print('started unregistering')
			self.subSPose.unregister()



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
		print('joint command=', jt.points)	
		self.pubTrajD.publish(jt)
		del p.positions[:]
		del jt.points[:]



	def jointPosReset(self,t=1):
		jt = Float64MultiArray()
		#jt.data = [0.0, (-np.pi)/4, 0.0, (np.pi)/2, 0.0, (-np.pi)/3, 0.0]
		jt.data = [0.0, 0.0, 0.0, -0.1, 0.0, 0.0, 0.8]
 
		print('traj command=', jt)
		self.pubTrajD.publish(jt)


	def jointPosCommand(self,t=1):
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


		if self.Reset == True:
			n = 1
			dJ1 = 0.0
			dJ2 = 0.0
			dJ3 = 0.0
			dJ4 = -0.1
			dJ5 = 0.0
			dJ6 = 0.0
			dJ7 = 0.8
		else:
			n = 10  # it works after* shutdown, but shut when 1st published
			dJ1 = 0.0
			dJ2 = -np.pi/3
			dJ3 = 0.0
			dJ4 = -np.pi/2
			dJ5 = 0.0
			dJ6 = np.pi/3
			dJ7 = 0.0

		J1 = np.repeat(dJ1,n)
		J2 = np.linspace(0,dJ2,n)
		J3 = np.repeat(dJ3,n)
		J4 = np.linspace(0,dJ4,n)
		J5 = np.repeat(dJ5,n)
		J6 = np.linspace(0,dJ6,n)
		J7 = np.repeat(dJ7,n)

		
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

			p.time_from_start = rospy.Duration.from_sec(dt[i]+0.5)  # time_from_start is the point in time at which that TrajectoryPoint should be executed.

			jt.points.append(p)
		print('traj command=', jt)
		self.pubTrajD.publish(jt)
		
		del p.positions[:]
		del jt.points[:]





if __name__ == "__main__":

	intprompV2_generator = promp_pub()
	while len(intprompV2_generator.q0) == 0:
		rospy.loginfo('I am still waiting to get robot init state')



	data = np.load('./results/bounds/latest/seqOpt_smooth0_obs1_max10_3cmsafety_20cmNeighdist/traj_opt_final_sat.npz')
	print('data=', data.shape)
	#traj_init = data[0]
	prompOpt = data

	#if intprompV2_generator.sat == True:
	#	prompOpt = np.load('./results/push_obs_thres_low/traj_opt_sat.npz')

	while not rospy.is_shutdown():
		if intprompV2_generator.Reset == True:
			intprompV2_generator.jointPosCommand(t=5)
			rospy.loginfo('Joint config reset finished')
		else:
			intprompV2_generator.jointTrajectoryCommand(prompOpt, t=1000.0)

