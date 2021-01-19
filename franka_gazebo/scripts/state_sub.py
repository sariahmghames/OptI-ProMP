#!/usr/bin/env python
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import rospy, math, time
import numpy as np
import matplotlib.pyplot as plt
from gazebo_msgs.msg import ModelState, LinkState
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.srv import SetLinkState, GetLinkState, SetLinkProperties
from gazebo_msgs.srv import GetModelProperties, GetWorldProperties
from gazebo_msgs.srv import DeleteModel
from control_msgs.msg import JointControllerState
from control_msgs.msg import FollowJointTrajectoryActionFeedback
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
import scipy
from scipy import signal
import tf



class Get_Clusters():

	def __init__(self,):
		rospy.init_node('register_models')
		self.Pi = np.pi
		self.color = ['Gazebo/Red', 'Gazebo/green']
		self.robot_name = 'panda_arm'
		self.world_client = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
		self.model_client = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
		self.set_link_property = rospy.ServiceProxy('/gazebo/set_link_properties', SetLinkProperties)
		self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		self.get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
		self.pubSPose = rospy.Publisher('/straws_states', PoseArray, queue_size=10)
		self.pubS1Pose = rospy.Publisher('/straw_states', PoseStamped, queue_size=10)
		self.pubSurrPose = rospy.Publisher('/surr_states', PoseArray, queue_size=10)
		self.panda_links = []
		self.panda_state = []
		self.gripper_links = []
		self.gripper_state = []
		self.straw_links = []
		self.straw_state = [] 
		self.goal_state_w = []
		### DEPRECATED
		self.world_models = ['tray_p2_8', 'straw_cluster7', 'straw_cluster8', 'straw_cluster11', 'straw_cluster1', 'straw_cluster2', 'straw_cluster3', 'straw_cluster4', 'panda']



	### DEPRICATED
	def get_body_names(self, model): 
		# rosservice call gazebo/get_world_properties
		

		panda =  {'body_names':['camera_bottom_screw_frame', 'panda_arm_link0', 'panda_arm_link1', 'panda_arm_link2', 'panda_arm_link3','panda_arm_link4', 'panda_arm_link5', 'panda_arm_link6', 'panda_arm_link7', 'gripper_leftfinger','gripper_rightfinger']} 
		clusters = {'straw_cluster1':{}, 'straw_cluster2':{}, 'straw_cluster3':{}, 'straw_cluster4':{}, 'straw_cluster7':{}, 'straw_cluster8':{}, 'straw_cluster11':{} }
		clusters['straw_cluster1'] = {'body_names':['/yball_stem11', '/xball_stem11', '/straw11', '/yball_stem12', '/xball_stem12', '/straw12',
  									'/yball_stem13', '/xball_stem13', '/straw13', '/yball_stem14', '/xball_stem14', '/straw14',
  									'/yball_stem15', '/xball_stem15', '/straw15']}
		clusters['straw_cluster2'] =  {'body_names':['/yball_stem21', '/xball_stem21', '/straw21', '/yball_stem22', '/xball_stem22', '/straw22',
  									'/yball_stem23', '/xball_stem23', '/straw23', '/yball_stem24', '/xball_stem24', '/straw24',
  									'/yball_stem25', '/xball_stem25', '/straw25','/yball_stem26', '/xball_stem26', '/straw26','/yball_stem27', '/xball_stem27', '/straw27','/yball_stem28', '/xball_stem28', '/straw28', 
  									'/yball_stem29', '/xball_stem29', '/straw29', '/yball_stem210', '/xball_stem210', '/straw210']}
		clusters['straw_cluster3'] =  {'body_names':['/yball_stem31', '/xball_stem31', '/straw31', '/yball_stem32', '/xball_stem32', '/straw32',
  									'/yball_stem33', '/xball_stem33', '/straw33', '/yball_stem34', '/xball_stem34', '/straw34',
  									'/yball_stem35', '/xball_stem35', '/straw35','/yball_stem36', '/xball_stem36', '/straw36','/yball_stem37', '/xball_stem37', '/straw37','/yball_stem38', '/xball_stem38', '/straw38', '/yball_stem39', '/xball_stem39', '/straw39', 
  									'/yball_stem310', '/xball_stem310', '/straw310', '/yball_stem311', '/xball_stem311', '/straw311', '/yball_stem312', '/xball_stem312', '/straw312', '/yball_stem313', '/xball_stem313', '/straw313', 
  									'/yball_stem314', '/xball_stem314', '/straw314', '/yball_stem315', '/xball_stem315', '/straw315']}
		clusters['straw_cluster4'] =  {'body_names':['/yball_stem41', '/xball_stem41', '/straw41', '/yball_stem42', '/xball_stem42', '/straw42',
  									'/yball_stem43', '/xball_stem43', '/straw43', '/yball_stem44', '/xball_stem44', '/straw44',
  									'/yball_stem45', '/xball_stem45', '/straw45','/yball_stem46', '/xball_stem46', '/straw46','/yball_stem47', '/xball_stem47', '/straw47','/yball_stem48', '/xball_stem48', '/straw48', 
  									'/yball_stem49', '/xball_stem49', '/straw49', 
  									'/yball_stem410', '/xball_stem410', '/straw410', '/yball_stem411', '/xball_stem411', '/straw411','/yball_stem412', '/xball_stem412', '/straw412', '/yball_stem413', '/xball_stem413', '/straw413', 
  									'/yball_stem414', '/xball_stem414', '/straw414', '/yball_stem415', '/xball_stem415', '/straw415', '/yball_stem416', '/xball_stem416', '/straw416',  '/yball_stem417', '/xball_stem417', '/straw417'
  									 '/yball_stem418', '/xball_stem418', '/straw418',  '/yball_stem419', '/xball_stem419', '/straw419', '/yball_stem420', '/xball_stem420', '/straw420']}
		clusters['straw_cluster7'] = {'body_names':['/yball_stem71', '/xball_stem71', '/straw71', '/yball_stem72', '/xball_stem72', '/straw72']}
		clusters['straw_cluster8'] = {'body_names':['/yball_stem81', '/xball_stem81', '/straw81', '/yball_stem82', '/xball_stem82', '/straw82', '/yball_stem83','/xball_stem83', '/straw83']}
		clusters['straw_cluster11'] = {'body_names':['/yball_stem111', '/xball_stem111', '/straw111', '/yball_stem112', '/xball_stem112', '/straw112',
  									'/yball_stem113', '/xball_stem113', '/straw113', '/yball_stem114', '/xball_stem114', '/straw114',
  									'/yball_stem115', '/xball_stem115', '/straw115']}

  		print('clusters=', clusters)
  		if 'panda' in model or 'panda_arm' in model:
  			return panda['body_names']
  		elif 'cluster' in model or 'straw' in model:
  			ind, char, dig_check = self.check_string(model)
  			var = 'straw_cluster'+ str(char)
			#print('var=',var)
			for key in clusters:
				print('key=', key)
				if key == var:
					return clusters[key]['body_names'] 
  		else:
  			return None



	def check_string(self, string):
		for ind, char in enumerate(string):
			dig_check = char.isdigit()
			if dig_check == True:
				return ind, char, dig_check



	def obj_in_base(self, obj_pose_w, base_pose_w):
		for obj in self.straw:
			self.goal_state_w.append(self.link_state(obj, "panda_arm_link0"))
		return



	def color_tag(self, straw):
		if '81' in straw or '71' in straw or '111' in straw or '113' in straw or 'straw11' in straw or 'straw12' in straw or 'straw13' in straw or 'straw21' in straw or 'straw23' in straw or 'straw26' in straw or 'straw31' in straw or 'straw33' in straw or \
			'straw35' in straw or  'straw38' in straw or 'straw41' in straw or 'straw43' in straw or 'straw45' in straw or 'straw48' in straw:
			return 'Red'
		else:
			return 'Green'



	def store_gazebo_models(self,):

		world_properties = rospy.wait_for_service('/gazebo/get_world_properties')
		try:
			resp_world = self.world_client()
		except rospy.ServiceException as exc:
		 	print("Service get_world_properties did not process request: " + str(exc))
		world_models = resp_world.model_names
		#print('world_models=', world_models)
		
		for model in world_models:
			#print('model is=',model)
			model_properties = rospy.wait_for_service('/gazebo/get_model_properties')
			try:
				model_prop = self.model_client(model)
			except rospy.ServiceException as exc:
		 		print("Service get_model_properties did not process request: " + str(exc))
			body_names = model_prop.body_names
			#print('bodies=', bodies)
			if 'panda' in model or 'panda_arm' in model or 'cluster' in model or 'panda' in model or 'panda_arm' in model or 'straw' in model:
				#joint_names = model.joint_names

				for link_ind, link in enumerate(body_names):
					if 'panda' in link or 'panda_arm' in link or 'cluster' in link or 'straw' in link:
						print('link=', link)
						print('model=', model)
						try:
							self.link_state = self.get_link_state(link, 'panda_arm_link0')
						except rospy.ServiceException as exc:
		 					print("Service get_link_state did not process request: " + str(exc))
						#print('link_state=',self.link_state)
						ind, dig, dig_check = self.check_string(link)
						link_orient_quat = self.link_state.link_state.pose.orientation
						link_position = self.link_state.link_state.pose.position
						link_vel_lin = self.link_state.link_state.twist.linear
						link_vel_ang = self.link_state.link_state.twist.angular
						## Get euler r p y from quat ##
						#### here #####
						euler = tf.transformations.euler_from_quaternion([link_orient_quat.x, link_orient_quat.y, link_orient_quat.z, link_orient_quat.w], axes='sxyz')
						if 'panda' in link or 'panda_arm' in link:
							self.panda_links.append(link)
							self.panda_state.append([link_position.x, link_position.y, link_position.z,euler[1]])
						elif 'finger' in link:
							self.gripper_links.append(link)
							self.gripper_state.append([link_position.x, link_position.y, link_position.z,euler[1]])
						elif 'straw' in link or 'cluster' in link:
							self.straw_links.append(link)
							self.straw_state.append([link_position.x, link_position.y, link_position.z,euler[1]])



	def straw8_pub(self,):
		msg = PoseArray()
		nb = 0
		pose_el = Pose()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = 'panda_arm_link0'
		for sind, straw in enumerate(self.straw_links):
			if ('straw81' in straw or 'straw82' in straw or 'straw83' in straw):
				print('selected straw8 is=', straw)
				print('selected straw8 state is=', self.straw_state[sind])
				pose_el.position.x = self.straw_state[sind][0] 
				pose_el.position.y = self.straw_state[sind][1]
				pose_el.position.z = self.straw_state[sind][2]
				pose_el.orientation.x = 0
				pose_el.orientation.y = 0
				pose_el.orientation.z = 0
				pose_el.orientation.w = 1
				print('pose_el is=', pose_el)
				msg.poses.extend([pose_el])
				nb += 1

		print('msg=', msg)
		self.pubSPose.publish(msg)



	def straw_pub(self,):
		msg = PoseStamped()
		color = 'Green'
		while color != 'Red':
			straw = np.random.choice(self.straw_links)
			color = color_tag(straw)

		sind = np.where(self.straw_links == straw)

		msg.pose.position.x = self.straw_state[sind][0]
		msg.pose.position.y = self.straw_state[sind][1]
		msg.pose.position.z = self.straw_state[sind][2]
		msg.pose.orientation.x = 0
		msg.pose.orientation.y = 0
		msg.pose.orientation.z = 0
		msg.pose.orientation.w = 1
		self.surr_pub(straw, sind)
		self.pub1SPose.publish(msg)



	def surr_pub(self, straw, sind):
		msg = PoseArray()
		surr_links = self.straw_links - straw
		surr_state = self.straw_state - self.straw_state[sind]

		msg.poses = surr_state
		self.pubSurrPose.publish(msg)



	def print_stored_poses(self,):
		print('straw poses=', self.straw_state)


# def cleanup():
# 	for act_EE_pos in act_EEtraj_pos:
# 		XeeA.append(act_EE_pos[0])
# 		YeeA.append(act_EE_pos[1])
# 		ZeeA.append(act_EE_pos[2])


# 	for des_EE_pos in des_EEtraj_pos:
# 		#des_EE_pos = np.squeeze(np.array([des_EEtraj_pos[desp]]))
# 		#des_EE_pos =  des_EE_pos.reshape((1, 3))
# 		XeeD.append(des_EE_pos[0])
# 		YeeD.append(des_EE_pos[1])
# 		ZeeD.append(des_EE_pos[2])

# 	smooth_posD = signal.medfilt(np.array([XeeD, YeeD, ZeeD]), kernel_size=1)
# 	print('smooth=', smooth_posD.shape)



if __name__ == '__main__':
	#counter = 0
	get_clusters = Get_Clusters()
	get_clusters.store_gazebo_models()
	get_clusters.straw8_pub()
	#sub = rospy.Subscriber('/panda_arm/arm_traj_controller/follow_joint_trajectory/feedback', FollowJointTrajectoryActionFeedback, joint_state_callback)
	rospy.spin()
	#rospy.on_shutdown(get_clusters.print_stored_poses)