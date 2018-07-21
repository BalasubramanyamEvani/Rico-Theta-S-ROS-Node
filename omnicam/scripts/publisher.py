#!/usr/bin/env python

'''

@Author:Balasubramanyam Evani
Manipal University Jaipur

Node publishes the left and right dual fish eye images obtained from Rico Theta S camera and the rectified images using parameters 
obtained from OcamCalib

'''

## Importing required libraries

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from ocamFunctions import get_ocam_model, create_perspective_undistortion_LUT ## other than persepective_undistortion there is panaromic undistortion, see ocamFunctions.py
from omnicam.msg import Omni

class Converter(object):

	def __init__(self):
		'''
		Publisher initializations
		'''
		self.out_right = rospy.Publisher('omni_right/image_raw',Image, queue_size = 10)
		self.out_left = rospy.Publisher('omni_left/image_raw',Image , queue_size = 10)
		self.rect_right = rospy.Publisher('omni_right/rect_image' , Image , queue_size = 10)
		self.rect_left = rospy.Publisher('omni_left/rect_image' , Image , queue_size = 10)
		self.left_cam_calib_info = rospy.Publisher('omni_left_info' , Omni, queue_size = 10)
		self.right_cam_calib_info = rospy.Publisher('omni_right_info' , Omni, queue_size = 10)
		'''
		Omni msg type object
		'''		
		self.msg_left = Omni()
		self.msg_right = Omni()

		self.bridge = CvBridge()				## Cvbridge object
		
		'''

		Reading the calibration results
	
		'''
		self.right_path = '../Results_OcamCalib/calib_results_right.txt'
		self.left_path = '../Results_OcamCalib/calib_results_left.txt'

		self.ocam_model_right = get_ocam_model(self.right_path)
		self.ocam_model_left = get_ocam_model(self.left_path)

		self.msg_left.len_pol = self.ocam_model_left['length_pol']
		self.msg_left.pols = self.ocam_model_left['pols']
		self.msg_left.len_invpol = self.ocam_model_left['length_invpol']
		self.msg_left.invpols = self.ocam_model_left['inv_pols']
		self.msg_left.cx = self.ocam_model_left['xc']
		self.msg_left.cy = self.ocam_model_left['yc']
		self.msg_left.height = self.ocam_model_left['height']
		self.msg_left.width = self.ocam_model_left['width']

		self.msg_right.len_pol = self.ocam_model_right['length_pol']
		self.msg_right.pols = self.ocam_model_right['pols']
		self.msg_right.len_invpol = self.ocam_model_right['length_invpol']
		self.msg_right.invpols = self.ocam_model_right['inv_pols']
		self.msg_right.cx = self.ocam_model_right['xc']
		self.msg_right.cy = self.ocam_model_right['yc']
		self.msg_right.height = self.ocam_model_right['height']
		self.msg_right.width = self.ocam_model_right['width']

		self.scalingFactor = 3.2
		(self.mapx_right , self.mapy_right) = create_perspective_undistortion_LUT(self.ocam_model_right , self.scalingFactor)
		(self.mapx_left , self.mapy_left) = create_perspective_undistortion_LUT(self.ocam_model_left , self.scalingFactor)

		self.mapx_right = self.mapx_right.astype("float32")
		self.mapy_right = self.mapy_right.astype("float32")

		self.mapx_left = self.mapx_left.astype("float32")
		self.mapy_left = self.mapy_left.astype("float32")
		
		rospy.loginfo("Omni Cam Node Starting Publishing") 		## logging info stating the node has started publishing

		self.image_sub = rospy.Subscriber('/omni_cam/image_raw',Image,self.callback) ## subscribes to topic mentioned

	def callback(self,data):						## callback function of image_sub

		time = data.header.stamp					## storing timestamp of orig image, to be used for left and right image header time stamp
		
		'''
		Conversion of Image msg to cv, also rotating the image, assuming the came in upright, if not remove the rotation part
		'''
		cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")	
		right , left = self.separate(cv_image)
		## rotation part for left image		
		scale = 1.0
		rows , cols = left.shape[:2]
		M1 = cv2.getRotationMatrix2D((cols/2, rows/2) , -90 , scale)
		cos = abs(M1[0,0])
		sin = abs(M1[0,1])
		nw = int((rows*sin) + (cols*cos))
		nh = int((rows*cos) + (cols*sin))
		M1[0,2] += (nw/2) - cols/2
		M1[1,2] += (nh/2) - rows/2
		left = cv2.warpAffine(left , M1 , (nw , nh))
		
		## rotation part for right image
		rows , cols = right.shape[:2]
		M2 = cv2.getRotationMatrix2D((cols/2 , rows/2) , 90 , scale)
		cos = abs(M2[0,0])
		sin = abs(M2[0,1])
		nw = int((rows*sin) + (cols*cos))
		nh = int((rows*cos) + (cols*sin))
		M2[0,2] += (nw/2) - cols/2
		M2[1,2] += (nh/2) - rows/2
		right = cv2.warpAffine(right , M2 , (cols , rows))

		try:
			'''
			Sending the Image msg
			'''
			omni_left = self.bridge.cv2_to_imgmsg(left,"bgr8")	
			omni_left.header.stamp = time
			omni_left.header.frame_id = "omni_left"

			omni_right = self.bridge.cv2_to_imgmsg(right,"bgr8")
			omni_right.header.stamp = omni_left.header.stamp
			omni_right.header.frame_id = "omni_right"

			self.out_right.publish(omni_right)
			self.out_left.publish(omni_left)


		except CvBridgeError as e:
			print e
		
		## conversion to grayscale for remapping with mapx,mapy obtained from the undistortion method

		gray_right = cv2.cvtColor(right , cv2.COLOR_BGR2GRAY)
		gray_left = cv2.cvtColor(left , cv2.COLOR_BGR2GRAY)


		rect_left = cv2.remap(gray_left , self.mapx_left , self.mapy_left , cv2.INTER_LINEAR)
		rect_right = cv2.remap(gray_right , self.mapx_right , self.mapy_right , cv2.INTER_LINEAR)


		try:
			'''
			conversion to BGR, it does not change the gray image to color, just adds the same value to all 3 channels,
			image view node does not accept the format if not converted to BGR.
			'''
			color_left = cv2.cvtColor(rect_left , cv2.COLOR_GRAY2BGR)
			color_right = cv2.cvtColor(rect_right , cv2.COLOR_GRAY2BGR)

			rectified_left = self.bridge.cv2_to_imgmsg(color_left,"bgr8")
			rectified_left.header.stamp = time
			rectified_left.header.frame_id = "omni_rect_left"

			rectified_right = self.bridge.cv2_to_imgmsg(left,"bgr8")
			rectified_right.header.stamp = time
			rectified_right.header.frame_id = "omni_rect_right"

			self.rect_left.publish(rectified_left)
			self.rect_right.publish(rectified_right)

		except CvBridgeError as e:
			print e
		'''
		publishing the camera info, affine parameters and co-efficients
		'''
		self.left_cam_calib_info.publish(self.msg_left)
		self.right_cam_calib_info.publish(self.msg_right)

	## method implements separation of images
	def separate(self,img):

        	height,width = img.shape[:2]
        	start_row , start_col = int(0) , int(0)
        	end_row , end_col  = int(height) , int(width * .5)
        	cropped_left = img[start_row : end_row , start_col : end_col]

        	start_row_right , start_col_right = int(0) , int(width * .5)
        	end_row_right , end_col_right = int(height) , int(width)
        	cropped_right = img[start_row_right : end_row_right , start_col_right : end_col_right]
        	return cropped_right,cropped_left


## Main Function

if __name__ == '__main__':

	rospy.init_node('Rico_Theta_S' , anonymous = True) ## initialization of node
	rospy.loginfo("Starting OmniCam Node")		## log info stating the node is getting registered
	converter = Converter()				## converter class object
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Exiting ..")
