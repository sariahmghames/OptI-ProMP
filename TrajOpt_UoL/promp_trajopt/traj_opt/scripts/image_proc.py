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
	#erosion = cv2.erode(mask1,kernel1,iterations = 1)
	dilation = cv2.dilate(mask1,kernel2,iterations = 1)

	mask2= cv2.inRange(lab, adj_lower_Dgreen, adj_upper_Dgreen)
	mask3= cv2.inRange(lab, adj_lower_Lgreen, adj_upper_Lgreen)
	#erosion3 = cv2.erode(mask3,kernel2,iterations = 1)
	dilation3 = cv2.dilate(mask3,kernel2,iterations = 1)
	mask_pre = cv2.bitwise_or(mask1, mask2)
	mask = cv2.bitwise_or(mask_pre, dilation3)
	print('mask shape=', mask1.shape)
	res= cv2.bitwise_and(image_bgr, image_bgr, mask=mask)

	#cv2.imshow('Original image', image)
	#cv2.imshow('mask1_'+str(loop1)+'.jpg',dilation)
	#cv2.imshow('mask2_'+str(loop1)+'.jpg',mask2)
	#cv2.imshow('mask_'+str(loop1)+'.jpg',mask)
	cv2.imshow('res_'+str(loop1)+'.jpg',res)
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