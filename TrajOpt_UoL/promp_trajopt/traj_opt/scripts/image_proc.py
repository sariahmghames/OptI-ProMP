import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3
import cv2 # pip install opencv-python in venv 
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy
import numpy as np
from PIL import Image 
import glob
from skimage import io, color
import imageio
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.image as mpimg  
from color_histogram.io_util.image import loadRGB
from color_histogram.core.hist_1d import Hist1D
from color_histogram.core.hist_2d import Hist2D


image_list = [] # image rgb
image_rgb = [] # pixels rgb
loop1 = 0
loop2 = 0
min_rect_dim =6
max_rect_dim = 15


##############################################################################################################################################################################

## Draw color histogram using opencv
# histo is a graph or a plot that represents the distribution of the pixel intensities in an image
#  RGB color space: the intensity of a pixel is in the range [0,255]. When plotting the histogram we have the pixel intensity in the X-axis and the frequency in the Y-axis
# The returned value hist is a numpy.ndarray with shape (n_bins, 1) where hist[i][0] is the number of pixels having an intensity value in the range of the i-th bin.


def draw_image_histogram(image, channels, color='k'):
    hist = cv2.calcHist([image], channels, None, [256], [0, 256])
    plt.plot(hist, color=color)
    plt.xlim([0, 256])
    plt.xlabel('pixel intensity')
    plt.ylabel('frequency')


def show_grayscale_histogram(image):
    grayscale_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    draw_image_histogram(grayscale_image, [0])
    plt.show()



def show_color_histogram(image):
    for i, col in enumerate(['b', 'g', 'r']):
        draw_image_histogram(image, [i], color=col)
    plt.show()


##########################################################################################################################################


for filename in glob.glob('/home/sariah/intProMP_franka/src/TrajOpt_UoL/promp_trajopt/clusters/rgb/*.jpg'):
	im = Image.open(filename)
	image_list.append(im)
	rgb = plt.imread(filename)
	image_rgb.append(rgb)


## color histogram with opencv
# for im in image_list:
# 	im.show()
# 	lab = color.rgb2lab(im, illuminant='D65') # illuminant is related to whitepoint D65= (32, 79, -108) 
# 	#print('rgb space=', im.shape)
# 	print('lab space=', lab.shape)
# 	#lab.save('/home/sariah/intProMP_franka/src/TrajOpt_UoL/promp_trajopt/clusters/trans_lab/imglab'+str(loop)+'.jpg', 'JPEG')
# 	#print('saved lab image:', loop)
# 	show_color_histogram(image_rgb[loop])
# 	loop = loop +1


## load image and calc hist of Lab space with color_histogram package github

## LAB colorspace has 3 channels, L which goes from 0-100%, a which goes from -128 to 128 and b which goes from -128 to 128. OpenCV values in LAB for a uint8 image are as follows:
# L_cv = L_lab * 255/100 ; a_cv = a_lab + 128 ; b_cv = b_lab + 128

for filename in glob.glob('/home/sariah/intProMP_franka/src/TrajOpt_UoL/promp_trajopt/clusters/rgb/*.jpg'):
	#image = loadRGB(filename)
	image_bgr = cv2.imread(filename) 
	image_bgr_copy = np.copy(image_bgr)
	image_bgr_copy2 = np.copy(image_bgr)
	image_bgr_copy3 = np.copy(image_bgr)
	image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
	cv2.imshow('RGB image 1', image_bgr)
	image_list[loop1].show()

	img_gray = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2GRAY)
	cv2.imshow('Gray image 1', img_gray)

	histL = Hist1D(image_rgb, num_bins = 16, color_space='Lab', channel= 0) # colorspace = 'Lab', 'HSV' channel: 0(L or h), 1, 2(v) ; H: the color, s: how strong that color is, v: how bright that color is
	fig = plt.figure()
	ax= fig.add_subplot(111)
	histL.plot(ax)
	plt.show()

	hista = Hist1D(image_rgb, num_bins = 16, color_space='Lab', channel= 1) # colorspace = 'Lab', 'HSV' channel: 0(L or h), 1, 2(v) ; H: the color, s: how strong that color is, v: how bright that color is
	fig = plt.figure()
	ax= fig.add_subplot(111)
	hista.plot(ax)
	plt.show()
 
	histb = Hist1D(image_rgb, num_bins = 16, color_space='Lab', channel= 2) # colorspace = 'Lab', 'HSV' channel: 0(L or h), 1, 2(v) ; H: the color, s: how strong that color is, v: how bright that color is
	fig = plt.figure()
	ax= fig.add_subplot(111)
	histb.plot(ax)
	plt.show()

	lab = cv2.cvtColor(image_rgb,cv2.COLOR_RGB2LAB)
	hsv = cv2.cvtColor(image_rgb,cv2.COLOR_RGB2HSV)
	cv2.imwrite('/home/sariah/intProMP_franka/src/TrajOpt_UoL/promp_trajopt/clusters/trans_lab/lab_'+str(loop1)+'.jpg', lab )
	l, a, b = cv2.split(lab)
	adj_lower_red = np.array([23*255/100, 20+128, 25.7+128], dtype = "uint8")
	adj_upper_red = np.array([50.77*255/100, 60.9+128, 42.5+128], dtype = "uint8")
	adj_lower_Dgreen = np.array([-1.1*255/100, -31.3+128, 9.5+128])
	adj_upper_Dgreen = np.array([55*255/100, -15.8+128, 35+128]) # 61.2
	adj_lower_Lgreen = np.array([68.3*255/100, -40.2+128, 19.5+128])
	adj_upper_Lgreen = np.array([86.1*255/100, -5.1+128, 54.2+128])
	#lower_red_hsv = np.array([0.77, 0.12, 0.037])
	#upper_red_hsv = np.array([0.89, 0.24, 0.098])

	#ret,thresh_L = cv2.threshold(L,70,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU) # to get thresholding on an image space using cv2.threshold use the AND operator to join threshold of L, a and b channels, 70 here is the threshold  and 255 is max value of pixel intensity to associate to pixels of intensity above the threshold
	#ret,thresh_a = cv2.threshold(a,70,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
	#ret,thresh_b = cv2.threshold(b,70,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

	## Thresholding on the trans
	mask1= cv2.inRange(lab, adj_lower_red, adj_upper_red) # binary mask
	#kernel1 = np.ones((1,1),np.uint8)
	kernel2 = np.ones((2,2),np.uint8)
	kernel3 = np.ones((5,5),np.uint8) # to reconstruct straw crossed by stem
	kernel4 = np.ones((9,9),np.uint8)
	#erosion = cv2.erode(mask1,kernel1,iterations = 1)
	dilation_old = cv2.dilate(mask1,kernel2,iterations = 1) 
	dilation = cv2.dilate(mask1,kernel4,iterations = 1) # but dilation fill the space with real pixels color, try to look into how to fill with same pixels color as surrounding

	mask2= cv2.inRange(lab, adj_lower_Dgreen, adj_upper_Dgreen)
	dilation2 = cv2.dilate(mask2,kernel4,iterations = 1)
	mask3= cv2.inRange(lab, adj_lower_Lgreen, adj_upper_Lgreen)
	#erosion3 = cv2.erode(mask3,kernel2,iterations = 1)
	dilation3 = cv2.dilate(mask3,kernel2,iterations = 1)
	mask_pre = cv2.bitwise_or(mask1, mask2)
	mask = cv2.bitwise_or(mask_pre, dilation3)
	print('mask shape=', mask1.shape)
	res= cv2.bitwise_and(image_bgr, image_bgr, mask=mask)
	res_LG= cv2.bitwise_and(image_bgr, image_bgr, mask=dilation3)
	res_DG= cv2.bitwise_and(image_bgr, image_bgr, mask=mask2)
	res_R= cv2.bitwise_and(image_bgr, image_bgr, mask=dilation)

	#cv2.imshow('Original image', image)
	#cv2.imshow('mask1_'+str(loop1)+'.jpg',dilation)
	cv2.imshow('mask2_'+str(loop1)+'.jpg',mask2)
	#cv2.imshow('mask_'+str(loop1)+'.jpg',mask)
	cv2.imshow('res_'+str(loop1)+'.jpg',res)
	cv2.imshow('res_LG'+str(loop1)+'.jpg',res_LG)
	cv2.imshow('res_DG'+str(loop1)+'.jpg',res_DG)
	cv2.imshow('res_R'+str(loop1)+'.jpg',res_R)

	# Model Fitting LG
	lines = cv2.HoughLinesP(dilation3,1,np.pi/180,20, maxLineGap=40)  # 2nd and 3rd arg are rho and theta params accuracies, 4th arg is threshold, represents min length of line to be considered as line (in terms of nb of pixels)
	for line in lines:
		x1, y1, x2, y2 = line[0]
		cv2.line(image_bgr_copy,(x1,y1),(x2,y2),(0,255,0),2) # start pt, end point, color, thickness
		cv2.imwrite('stem_clusters'+str(loop1)+'.jpg', image_bgr_copy)

	cv2.imshow("model_fitting_LG", image_bgr_copy)


	# Model Fitting DG
	ret,thresh = cv2.threshold(dilation2,127,255,cv2.THRESH_BINARY)
	#cv2.imshow('threshold',thresh)
	contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) # cv2.CHAIN_APPROX_NONE (734 points) and second image shows the one with cv2.CHAIN_APPROX_SIMPLE (only 4 points). See, how much memory it saves!!!
	for cnt in contours:
		#cnt=contours[0]
		x,y,w,h = cv2.boundingRect(cnt)
		if w > 20 and h > 10 and w < 180 and h < 140:
			stem_rect = cv2.rectangle(image_bgr_copy2,(x,y),(x+w,y+h),(0,0,255),2)
			cv2.imwrite('lid_clusters'+str(loop1)+'.jpg', image_bgr_copy2)
		
	cv2.imshow("model_fitting_DG",image_bgr_copy2)



	# Model Fitting DG
	ret_R,thresh_R = cv2.threshold(dilation,127,255,cv2.THRESH_BINARY)
	#cv2.imshow('threshold',thresh)
	contours_R, hierarchy_R = cv2.findContours(thresh_R,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) # cv2.CHAIN_APPROX_NONE (734 points) and second image shows the one with cv2.CHAIN_APPROX_SIMPLE (only 4 points). See, how much memory it saves!!!
	if len(contours_R)>0 :
		for cnt_R in contours_R:
		# get the min area rect
			rect = cv2.minAreaRect(cnt_R)
			box = cv2.boxPoints(rect)
			# convert all coordinates floating point values to int
			box = np.int0(box)
			# draw a blue rectangle
			#cv2.drawContours(image_bgr_copy3, [box], 2, (255, 0, 0))

			# finally, get the min enclosing circle
			# Model Fitting Red
			(x, y), radius = cv2.minEnclosingCircle(cnt_R)
			# convert all values to int
			center = (int(x), int(y))
			radius = int(radius)
			if (radius > 10 and radius < 200):
				# and draw the circle in red
				straw_circles = cv2.circle(image_bgr_copy3, center, radius, (0, 0, 255), 2)
				cv2.imwrite('straw_clusters'+str(loop1)+'.jpg', image_bgr_copy3)


		cv2.imshow("model_fitting_R",image_bgr_copy3)


	cv2.waitKey(0)

	cv2.destroyAllWindows()

	#loop1 = loop1 +1


## 2D Histogram

# for filename in glob.glob('/home/sariah/intProMP_franka/src/TrajOpt_UoL/promp_trajopt/clusters/rgb/*.jpg'):
# 	image = loadRGB(filename)
# 	image_list[loop2].show()
# 	# 32 bins, hsv color space, target channels (h, s) ('hsv'[0], 'hsv'[1])
# 	hist2D = Hist2D(image, num_bins=32, color_space='Lab', channels=[0, 1])
# 	fig = plt.figure()
# 	ax = fig.add_subplot(111)
# 	hist2D.plot(ax)
# 	plt.show()
# 	loop2 = loop2 +1


















# References: https://lmcaraig.com/understanding-image-histograms-with-opencv
# https://github.com/tody411/ColorHistogram