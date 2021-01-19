#!/usr/bin/env python
# This scripts record online scene images from rgb-d camera in simulation, for CNN network object detection

import rospy, math, time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from numbers import Number
from matplotlib.lines import Line2D
from sensor_msgs.msg import JointState
import os
from datetime import datetime
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import roslib


class data_collection():


	def __init__(self,):
		rospy.init_node('data_collection', anonymous=True)
		self.rate = rospy.Rate(10)
		## Cam properties
		self.colorpp =  np.array([6.39151982e+02,3.89859506e+02])
		self.colorff =  np.array([1.05139286e+03,1.05053887e+03])		
   		## Uncomment the 2 subscribers if the camera is connected
   		#self.subDepth = rospy.Subscriber("/camera/aligned_depth_to_color/depth_raw", Image, self.callback1=get_depth, queue_size=1)
   		self.subDepth = rospy.Subscriber("/camera/depth/image_raw", Image, callback=self.get_depth, queue_size=1)
   		self.subColor = rospy.Subscriber("/camera/color/image_raw", Image, callback=self.get_rgb, queue_size=1)
   		# Use cv_bridge() to convert the ROS image to OpenCV format
   		self.bridge = CvBridge()
   		self.sample_depth = 0
   		self.sample_rgb = 0
   		self.x_init = 0
   		self.y_init = -0-5

		################################################################


	def get_rgb(self, ros_image):
		self.sample_rgb +=1 
		try:
		#Convert the depth image using the default passthrough encoding
			rgb_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")

		except CvBridgeError, e:
			print e

		#print('rgb shape =', rgb_image.shape)
		rows, cols, channels = rgb_image.shape
		#cv2.imshow("Rgb Image window", rgb_image)
		cv2.waitKey(1000)
		if self.sample_rgb <= 10 :
			cv2.imwrite("/home/sariah/intProMP_franka/src/franka_gazebo/dataset/color/rgb_x{}_spl{}.jpg".format(self.x_init,self.sample_rgb), cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB))
			#Convert the depth image to a Numpy array
			img_array = np.array(rgb_image, dtype=np.float32)

			rospy.loginfo("saved rgb img")


	def get_depth(self, msg_depth):
		self.sample_depth +=1 
		try:
		#Convert the depth image using the default passthrough encoding
			depth_image = self.bridge.imgmsg_to_cv2(msg_depth, "32FC1")

		except CvBridgeError, e:
		          print e

		#print('depth shape =', depth_image.shape)
		rows, cols = depth_image.shape
		#cv2.imshow("Depth Image window", depth_image)
		cv2.waitKey(1000)
		if self.sample_depth <= 10 :
			cv2.imwrite("/home/sariah/intProMP_franka/src/franka_gazebo/dataset/depth/depth_x{}_spl{}.jpg".format(self.x_init,self.sample_depth), depth_image)
			#Convert the depth image to a Numpy array
			depth_array = np.array(depth_image, dtype=np.float32)
			rospy.loginfo("saved depth image")


	# def vectorized_pixe_to_point(depth_raw,color_raw,annotation,color_principal_point,color_focal_length):  # get point cloud from rgb and depth image, "color_principal_point,color_focal_length" are camera intrinsic parameters, 'annotation' is label2.png
	#     depth_raw = depth_raw/1000  # /1000 to get the depth of each pixel in meters, depth_raw shape is (720,1280)
	#     depth_raw2 = np.expand_dims(depth_raw,2)  # np.expand_dims(a,axis), 2 is along axis z, dim will be 1 along 3rd axis, so (720,1280,1)
	#     depth_raw3 = np.concatenate([depth_raw2,depth_raw2],2) # 2 copies of depth_raw2 are concatenated one after the other along the 3rd dim , so shape is (720,1280,2)
	#     x =  np.expand_dims(np.repeat(np.expand_dims(np.arange(depth_raw.shape[1]),0),depth_raw.shape[0],0),2) # np.repeat(array, nb of rep, axis),  np.arange(depth_raw.shape[1]) returns : array([0,1,2...1279]), expand along axis 0 gives shape of ( 1,1280) so array([[0,1,2....1279]]) then will be copied 720times along axis 0, x.shape gives (720,1280,1), it assigns the column nb to each pixel ... means x values
	#     y = np.arange(depth_raw.shape[0])  # array([0,1,2...719])
	#     y = np.expand_dims(y,1) # array([[0], [1], ...[719]]) , y.shape is (720,1)
	#     y = np.expand_dims(np.repeat(y,depth_raw.shape[1],1),2) #y.shape is (720,1280,1), assigns row nb to each pixel , means y value
	#     z = np.concatenate([x,y],2) # z.shape (720,1280,2)
	#     points = (z - color_principal_point) * depth_raw3 / color_focal_length  # multiply x and y values of each pixel by the depth value, considering intrinsic params, points.shape(720,1280,2)
	#     points = np.concatenate([points, depth_raw2],2) # add depth array as 3rd layer in points 3D array, points.shape (720,1280,3), 1st layer is x, 2nd is y and 3rd is depth
	#     points =  np.reshape(points,(points.shape[0]*points.shape[1],points.shape[2])) # got 3 columns , in 1st we have all x values aligned in 1 col, in 2nd we have y values, and 3rd we have depth values
	#     points = points * np.array([1,-1.0,-1.0])  # np.array([1,-1.0,-1.0]) ?
	#     colors = np.reshape(color_raw,(color_raw.shape[0]*color_raw.shape[1],color_raw.shape[2])) # 720*1280 ... will get 1 colomn of rgb value for each pixel
	#     annotation = np.reshape(annotation,(annotation.shape[0]*annotation.shape[1]))
	#     annotation = annotation[np.nonzero(points[:,2])] # indices of pixels with nonzero depth
	#     colors = colors [np.nonzero(points[:,2])]
	#     points = points[np.nonzero(points[:,2])]
	#     return (points,colors,annotation )


	# def get_pcd(self,):
	# 	#################################################################################################################

	# 	depth_raw = png.Reader("./pcl_Justin/annotated_straw/depth.png")
	# 	_,_,pngData,_= depth_raw.asDirect() # Like the read() method this method returns a 4-tuple:(width, height, pixel values, meta)
	# 	depth_raw = np.vstack(list(pngData))
	# 	color = np.asarray(cv2.imread("./pcl_Justin/annotated_straw/rgb.png"))/255
	# 	annotation = np.asarray(Image.open("./pcl_Justin/annotated_straw/label2.png"))
	# 	points, colors, annotation = vectorized_pixe_to_point(depth_raw,color,annotation,self.colorpp,self.colorff)
	# 	classa = {} # a dictionary, to store how many classes are in the image annotated
	# 	for k,a in enumerate(annotation): #k is the element index and a is the element value in annotation array
	# 	    if(a!=0): # everything different than the background
	# 	        if(not a in classa.keys()):
	# 	            classa[a] = [k] # a is exactly a key stored in the dict , to which we assign values k: the indices of array annotation that have value a, without []
	# 	        else:
	# 	            val  = classa[a] # val will b the value existing in the key a 
	# 	            val.append(k)  # append a value (index of point with key a) to the key
	# 	            classa[a] = val

	# 	print('keys=',classa.keys())
	# 	desired_straw_annot = 12
	# 	annotation_SingStr = desired_straw_annot 
	# 	Ind_SingStr = classa[desired_straw_annot]
	# 	points_SingStr = points[Ind_SingStr]
	# 	colors_SingStr = colors[Ind_SingStr]
	# 	return (points_SingStr,colors_SingStr,annotation_SingStr)


	# def pcl_callback(self,):
	# 	points_SingStr,colors_SingStr,annotation_SingStr = get_pcd()




if __name__ == '__main__':
	try:
		init_data = data_collection()
	except rospy.ROSInterruptException:
		pass

	while not rospy.is_shutdown():
	    rospy.spin()
	    init_data.rate.sleep()

