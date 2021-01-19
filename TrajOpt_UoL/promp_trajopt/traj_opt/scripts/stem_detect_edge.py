import sys
import cv2 # pip install opencv-python in venv 
import numpy as np
from PIL import Image 
import glob
from skimage import io, color
import imageio
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.image as mpimg  

loop = 1

for filename in glob.glob('/home/sariah/intProMP_franka/src/TrajOpt_UoL/promp_trajopt/clusters/rgb/*.jpg'):
	img_bgr = cv2.imread(filename) 
	img = cv2.imread(filename,0)
	edges = cv2.Canny(img,200,500, apertureSize = 3)
	#print('edges shape=', edges.shape)

	# Model Fitting 
	lines = cv2.HoughLinesP(edges,1,np.pi/180,30, maxLineGap=20)  # 2nd and 3rd arg are rho and theta params accuracies, 4th arg is threshold, represents min length of line to be considered as line (in terms of nb of pixels)
	blank_image = np.zeros((img.shape[0], img.shape[1]), np.uint8)
	for line in lines:
		x1, y1, x2, y2 = line[0]
		cv2.line(img_bgr,(x1,y1),(x2,y2),(0,255,0),2) # start pt, end point, color, thickness

    # cv2.imshow("out", blank_image)
    # cv2.waitKey(0)
	cv2.imwrite('houghlines_clusters'+str(loop)+'.jpg', img_bgr)  
	loop +=1

	plt.subplot(121),plt.imshow(img,cmap = 'gray')
	plt.title('Original Image'), plt.xticks([]), plt.yticks([])
	plt.subplot(122),plt.imshow(edges,cmap = 'gray')
	plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
	plt.show()



## For cv2.HoughLines(edges, rho, theta, threshold), do:
	# for x in range(0, len(lines)):    
	# 	for rho, theta in lines[x]:# iterate over params rho and theta
	# 	    a = np.cos(theta)
	# 	    b = np.sin(theta)
	# 	    x0 = a*rho
	# 	    y0 = b*rho
	# 	    x1 = int(x0 + 1000*(-b))
	# 	    y1 = int(y0 + 1000*(a))
	# 	    x2 = int(x0 - 1000*(-b))
	# 	    y2 = int(y0 - 1000*(a))