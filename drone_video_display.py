#!/usr/bin/env python

# A basic video display window for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This display window listens to the drone's video feeds and updates the display at regular intervals
# It also tracks the drone's status and any connection problems, displaying them in the window's status bar
# By default it includes no control functionality. The class can be extended to implement key or mouse listeners if required

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib
import rospy
import cv2

# Import the two types of messages we're interested in
from sensor_msgs.msg import Image    	 # for receiving the video feed
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
from ardrone_autonomy.srv import *

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock

# An enumeration of Drone Statuses
from drone_status import DroneStatus

# The GUI libraries
from PySide import QtCore, QtGui

# 2017-03-22 Import libraries from OpenCV
# OpenCV Bridge http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import String


# Some Constants
CONNECTION_CHECK_PERIOD = 250 #ms
GUI_UPDATE_PERIOD = 20 #ms
DETECT_RADIUS = 4 # the radius of the circle drawn when a tag is detected


class DroneVideoDisplay(QtGui.QMainWindow):
	StatusMessages = {
		DroneStatus.Emergency : 'Emergency',
		DroneStatus.Inited    : 'Initialized',
		DroneStatus.Landed    : 'Landed',
		DroneStatus.Flying    : 'Flying',
		DroneStatus.Hovering  : 'Hovering',
		DroneStatus.Test      : 'Test (?)',
		DroneStatus.TakingOff : 'Taking Off',
		DroneStatus.GotoHover : 'Going to Hover Mode',
		DroneStatus.Landing   : 'Landing',
		DroneStatus.Looping   : 'Looping (?)'
		}
	DisconnectedMessage = 'Disconnected'
	UnknownMessage = 'Unknown Status'


	def __init__(self):
		# Construct the parent class
		super(DroneVideoDisplay, self).__init__()

		# Setup our very basic GUI - a label which fills the whole window and holds our image
		self.setWindowTitle('AR.Drone Video Feed')
		self.imageBox = QtGui.QLabel(self)
		self.setCentralWidget(self.imageBox)

		#---------------------------------------------------------------------
    	# Hoop height publisher.
    	#---------------------------------------------------------------------
		self.pubHoopHeight  = rospy.Publisher('/ardrone/hoop_height', String, queue_size=10)

		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata,queue_size=1) 

		# Subscribe to the drone's video feed, calling self.ReceiveImage when a new frame is received
		self.subVideo   = rospy.Subscriber('/ardrone/image_raw',Image,self.ReceiveImage,queue_size=1)

		#---------------------------------------------------------------------
    	# Subscriber for whether to capture image or not.
    	#---------------------------------------------------------------------
		self.subTakePic = rospy.Subscriber('/ardrone/take_pic', String, self.SaveImage)

		#---------------------------------------------------------------------
    	# UAV location subscriber.
    	#---------------------------------------------------------------------
		self.model_name = 'ARDroneCarre'

		self.sub_vicon_data = rospy.Subscriber('/vicon/{0}/{0}'.format(self.model_name), TransformStamped, self.update_vicon_data)

		# Holds the image frame received from the drone and later processed by the GUI
		self.image = None
		self.imageLock = Lock()

		self.tags = []
		self.tagLock = Lock()

		# Holds the status message to be displayed on the next GUI update
		self.statusMessage = ''

		# Tracks whether we have received data since the last connection check
		# This works because data comes in at 50Hz but we're checking for a connection at 4Hz
		self.communicationSinceTimer = False
		self.connected = False

		# A timer to check whether we're still connected
		self.connectionTimer = QtCore.QTimer(self)
		self.connectionTimer.timeout.connect(self.ConnectionCallback)
		self.connectionTimer.start(CONNECTION_CHECK_PERIOD)

		# A timer to redraw the GUI
		self.redrawTimer = QtCore.QTimer(self)
		self.redrawTimer.timeout.connect(self.RedrawCallback)
		self.redrawTimer.start(GUI_UPDATE_PERIOD)

		# 2017-03-22 convert ROS images to OpenCV images
		self.bridge = CvBridge()

		# 2017-03-31 Lab 4 processing images variables
		self.processImages = False
		self.cv_output = None
		self.cv_img = None
		# detect hoop every [x] seconds, don't process too frequently as it may introduce latency
		self.commandTimer = rospy.Timer(rospy.Duration(0.5),self.HoopDetector)
		# color boundaries, in the form of (lower[B,G,R], upper[B,G,R]) 
		# TODO try and test out the color BGR values
		self.hoop_colorbounds = [([17, 15, 100], [50, 56, 200])] # TODO these values are placeholders

		#---------------------------------------------------------------------
    	# Initial variables for UAV location data.
    	#---------------------------------------------------------------------
		self.current_trans_x = 0
		self.current_trans_y = 0
		self.current_trans_z = 0
		self.current_rot_x = 0
		self.current_rot_y = 0
		self.current_rot_z = 0
		self.current_rot_w = 0
		self.pic_number = 1

	# Called every CONNECTION_CHECK_PERIOD ms, if we haven't received anything since the last callback, will assume we are having network troubles and display a message in the status bar
	def ConnectionCallback(self):
		self.connected = self.communicationSinceTimer
		self.communicationSinceTimer = False

	def RedrawCallback(self):
		if self.image is not None:
			# We have some issues with locking between the display thread and the ros messaging thread due to the size of the image, so we need to lock the resources
			self.imageLock.acquire()
			try:			
					# Convert the ROS image into a QImage which we can display
					if self.processImages == False:
						image = QtGui.QPixmap.fromImage(QtGui.QImage(self.image.data, self.image.width, self.image.height, QtGui.QImage.Format_RGB888))						
					# display processed image when processing is enabled
					else:
						if self.cv_output is not None:					
							# convert from openCV output cv_output image back to ROS image (Optional for visualization purposes)
							img_msg = self.bridge.cv2_to_imgmsg(self.cv_output, encoding="bgr8")
							# convert to QImage to be displayed
							image = QtGui.QPixmap.fromImage(QtGui.QImage(img_msg.data, img_msg.width, img_msg.height, QtGui.QImage.Format_RGB888))
						else:
							image = QtGui.QPixmap.fromImage(QtGui.QImage(self.image.data, self.image.width, self.image.height, QtGui.QImage.Format_RGB888))		
					
			finally:
				self.imageLock.release()

			# We could  do more processing (eg OpenCV) here if we wanted to, but for now lets just display the window.
			self.resize(image.width(),image.height())
			self.imageBox.setPixmap(image)

		# Update the status bar to show the current drone status & battery level
		self.statusBar().showMessage(self.statusMessage if self.connected else self.DisconnectedMessage)
		
		

	def ReceiveImage(self,data):
		# Indicate that new data has been received (thus we are connected)
		self.communicationSinceTimer = True

		# We have some issues with locking between the GUI update thread and the ROS messaging thread due to the size of the image, so we need to lock the resources
		self.imageLock.acquire()
		try:
			self.image = data # Save the ros image for processing by the display thread
			# 2017-03-22 we do not recommend saving images in this function as it might cause huge latency
		finally:
			self.imageLock.release()
			
	# 2017-03-22 sample function of saving images.
	# TODO feel free to modify, you could use a timer to capture the image at a certain rate, or modify the keyboard_controller.py to capture the image through a key
	def SaveImage(self, takePic_msg):
		"""Captures image when the takePic_msg prompts image capture."""
		# Check whether the msg data is not no.
		if takePic_msg.data != "no":
			# ensure not in the process of acquiring image
			if self.imageLock.acquire():
				try:
					# convert from ROS image to OpenCV image
					cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="bgr8")
				except CvBridgeError as e:
					print "Image conversion failed: %s" % e
				# TODO use cv2.imwrite function to write images to local disk
				# might want to consider storing the corresponding vehicle attitude/position to geolocate target

				# Sets file directory and image name to the one given from the msg.
				file_dir = "/home/tausif/Desktop/Project_test/" + takePic_msg.data

				# Saves image to the file directory chosen.
				cv2.imwrite(file_dir, cv_image)
				
				self.imageLock.release()


	# 2017-03-31 Lab 4 code ================================
	# the codes written here serve as a guideline, it is not required that you use the code. Feel free to modify.

	def EnableImageProcessing(self):  # called from KeyboardController Key_P
		self.processImages = True

	def DisableImageProcessing(self): # called from KeyboardController Key_P
		self.processImages = False
		self.pubHoopHeight.publish("no")

	def HoopDetector(self,event):
		# reference: http://www.pyimagesearch.com/2014/08/04/opencv-python-color-detection/
		self.pubHoopHeight.publish("no")
		if self.image is not None and self.processImages == True:
			if self.imageLock.acquire():
				try:
					# convert from ROS image to OpenCV image
					self.cv_img = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="bgr8")
				except CvBridgeError as e:
					print "Image conversion failed: %s" % e
				self.imageLock.release()

				#---------------------------------------------------------------------
    			# Sending the full image to the ProcessHoopImage function since
    			# colour isolation/masking techinique was abandoned.
    			#---------------------------------------------------------------------
				self.ProcessHoopImage(self.cv_img)
				# carry out color thresholding
				"""
				for (lower, upper) in self.hoop_colorbounds:
					lower = np.array(lower, dtype = "uint8")
					upper = np.array(upper, dtype = "uint8")
					# find the colors within the specified boundaries and apply mask
					# the mask shows values = 255 for image pixels that fall within the boundaries
					mask = cv2.inRange(self.cv_img, lower, upper)
					self.cv_output = cv2.bitwise_and(self.cv_img, self.cv_img, mask = mask)
					self.ProcessHoopImage(mask)
				"""

	
	def ProcessHoopImage(self,mask):
		# TODO algorithm to detect center of circular hoop based on pixel locations
		# suggested method to obtain pixel locations using numpy
		# https://docs.scipy.org/doc/numpy/reference/generated/numpy.where.html
		px_locations = np.where( mask > 0 )
		# will return (array_of_row_index, array_of_col_index) of the corresponding pixel location

		# A simple square kernel for morphing transformations.
		kernel = np.ones((5,5),np.uint8)

		# Adding an opening blur (removes some noise).
		image = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
		
		# Keeping original BGR image as output for saving (debugging purposes).
		output = image.copy()

		# Converting image to grayscale to be used by Hough Transform.
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

		# Applying the Hough Transform to the image. This transform specifically looks for circles.
		circles = cv2.HoughCircles(gray, cv2.cv.CV_HOUGH_GRADIENT, 5, 475, minRadius = 65, maxRadius = 0)
		
		# Ensure only 1 circles is found. Sometimes can pick up background noise as circles.
		if (circles != None) and (circles.shape[1] == 1):
			# Rounding and casting (x, y, r) of circles into an int.
			circles = np.round(circles[0, :]).astype("int")
		 
			# Drawing the circle found over the output image for saving purposes.
			for (x, y, r) in circles:
				# draw the circle in the output image, then draw a rectangle
				# corresponding to the centroid of the circle
				cv2.circle(output, (x, y), r, (0, 255, 0), 3)
				cv2.rectangle(output, (x - 3, y - 3), (x + 3, y + 3), (255, 0, 0), -1)
		 		
			# Getting current UAV location.
			x_now = self.current_trans_x
			y_now = self.current_trans_y
			z_now = self.current_trans_z

			# Calculating the UAV's distance to the hoop's plane
			hoop_dist = np.sqrt(np.power(x_now, 2) + np.power(y_now, 2))
			
			# Calculating the hoop plane's diadonal with respect to the camera's diagonal FOV.
			plane_diag = 2*((hoop_dist*np.sin(46*(np.pi/180)))/np.sin(44*(np.pi/180)))

			# Calculating x & y lengths in meters using pixel conversion.
			plane_x = plane_diag*(640/800)
			plane_y = plane_diag*(360/800)

			# Calculate the overall axes with respect to the current image's location.
			plane_x_axis = np.array([x_now - (plane_x/2), x_now + (plane_x/2)])
			plane_y_axis = np.array([y_now - (plane_y/2), y_now + (plane_y/2)])

			# Calculating the error in the UAV's current altitude and the hoop's centroid.
			# z_error = 0 means that both the UAV's altitude and hoop's centroid are at the center of the image.
			z_error = (180 - y)*(plane_y/360)
			z_new = z_now + z_error

			# Printing new altitude (debugging purposes).
			print(z_new)

			# Publishing new height.
			self.pubHoopHeight.publish(str(z_new))

			# Saving the initial image when the circle is first detected.
			"""
			if self.pic_number == 1:
				file_name = str(self.pic_number) + "_X" + str(round(self.x_now, 2)) + "_Y" + str(round(y_now, 2)) + "_Z" + str(round(z_now, 2)) + ".jpg"
				file_dir = "/home/tausif/aer1217/labs/src/aer1217_ardrone_vicon/scripts/" + file_name
				cv2.imwrite(file_dir, output)
				self.pic_number = self.pic_number + 1
			"""


		# If no circles, or more than 1 are found, then just publish "no", which desired_position uses to ignore changes.
		else:
			# Publishing whether a height has been determined or not.
			self.pubHoopHeight.publish("no")


	def update_vicon_data(self, vicon_data_msg):
		# Updating the current position of the UAV.
		
		(self.current_trans_x,
		self.current_trans_y,
		self.current_trans_z) = (vicon_data_msg.transform.translation.x,
								vicon_data_msg.transform.translation.y,
								vicon_data_msg.transform.translation.z)

		(self.current_rot_x,
		self.current_rot_y,
		self.current_rot_z,
		self.current_rot_w) = (vicon_data_msg.transform.rotation.x,
								vicon_data_msg.transform.rotation.y,
								vicon_data_msg.transform.rotation.z,
								vicon_data_msg.transform.rotation.w)

	# 2017-03-31 ============================================

	def ReceiveNavdata(self,navdata):
		# Indicate that new data has been received (thus we are connected)
		self.communicationSinceTimer = True

		# Update the message to be displayed
		msg = self.StatusMessages[navdata.state] if navdata.state in self.StatusMessages else self.UnknownMessage
		self.statusMessage = '{} (Battery: {}%)'.format(msg,int(navdata.batteryPercent))

		self.tagLock.acquire()
		try:
			if navdata.tags_count > 0:
				self.tags = [(navdata.tags_xc[i],navdata.tags_yc[i],navdata.tags_distance[i]) for i in range(0,navdata.tags_count)]
			else:
				self.tags = []
		finally:
			self.tagLock.release()

if __name__=='__main__':
	import sys
	rospy.init_node('ardrone_video_display')
	app = QtGui.QApplication(sys.argv)
	display = DroneVideoDisplay()
	display.show()
	status = app.exec_()
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)
