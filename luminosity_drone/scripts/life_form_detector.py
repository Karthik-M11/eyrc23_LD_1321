#!/usr/bin/env python3

'''

This python file runs a ROS-node of name drone_control which holds the position of Swift-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
					
								

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from swift_msgs.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import rospy
import time
import imutils
from imutils import contours
from skimage import measure
import numpy as np


def led_finder(image):
	# convert it to grayscale, and blur it
	# image=cv2.rotate(image,cv2.ROTATE_180)
	gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	blur_image = cv2.blur(gray_image, (11,11))
	# threshold the image to reveal light regions in the blurred image
	threshold_image = cv2.threshold(blur_image, 127, 255, cv2.THRESH_BINARY)[1]
	# perform a series of erosions and dilations to remove any small blobs of noise from the thresholded image
	threshold_image = cv2.erode(threshold_image, None, iterations=1)
	threshold_image = cv2.dilate(threshold_image, None, iterations=1)
	# perform a connected component analysis on the thresholded image, then initialize a mask to store only the "large" components
	label_image = measure.label(threshold_image, background=0, connectivity=None)
	mask = np.zeros(threshold_image.shape, dtype="uint8")
	# loop over the unique components
	for label in np.unique(label_image):
			# if this is the background label, ignore it
			if label == 0:
				continue
			# otherwise, construct the label mask and count the number of pixels
			labelMask = np.zeros(threshold_image.shape, dtype="uint8")
			labelMask = np.zeros(threshold_image.shape, dtype="uint8")
			labelMask[label_image == label] = 255
			numPixels = cv2.countNonZero(labelMask)
		# if the number of pixels in the component is sufficiently large, then add it to our mask of "large blobs"
			if numPixels > 200:
				mask = cv2.add(mask, labelMask)
	# find the contours in the mask, then sort them from left to right
	contour = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	contour = imutils.grab_contours(contour)
	# print(contour)
	if len(contour) == 0:
			pass
	else:
		contour = contours.sort_contours(contour)[0]
	# loop over the contours
	# Initialize lists to store centroid coordinates and area
	avg_centroid=[0,0]
	Centroid = []
	Area = []

	# print(contour[0])
	# Loop over the contours
	for (i, c) in enumerate(contour):
		# Calculate the area of the contour
			area = cv2.contourArea(c) 

		# Draw the bright spot on the image
			(x, y, w, h) = cv2.boundingRect(c)
			(cX, cY), radius = cv2.minEnclosingCircle(c)
			cv2.circle(image, (int(cX), int(cY)), int(radius),(0, 0, 255), 2)
			cv2.putText(image, "#{}".format(i + 1), (x, y - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)

			cx = (x+x+w)/2
			cy = (y+y+h)/2

		# Append centroid coordinates and area to the respective lists
			Area.append(area)
			Centroid.append((cx,cy))
			avg_centroid[0]+=cx
			avg_centroid[1]+=cy
	avg_centroid=list(map(lambda x:x/len(avg_centroid),avg_centroid))
	dimension=image.shape
	avg_centroid[0]/=dimension[1]
	avg_centroid[1]/=dimension[0]
	return len(Area),avg_centroid

class swift():
	"""docstring for swift"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [2,2,20] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
		#Declaring a cmd of message type swift_msgs and initializing values
		self.cmd = swift_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500
		self.current_frame=None
		self.led_no=0

		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters

		self.Kp = [12, 24.5, 28.8] #[22.5, 22.5, 28.8]   #[22.5, 0, 40.8]
		self.Ki = [0, 0, -0.01] #[0, 0, 0.174]   #[0, 0, 0.18575]
		self.Kd = [325.5, 325.5, 710] #[325.5, 325.5, 597.5]   #[325.5, 0, 750]
   
		#-----------------------Add other required variables for pid here ----------------------------------------------

		self.prev_error = [0,0,0]
		self.min_values = [1000, 1000, 1000]
		self.max_values = [2000, 2000, 2000]

		self.error = [0, 0, 0]
		self.integral_error = [0, 0, 0]
		self.derivative_error = [0, 0, 0]

		self.out_roll = [0, 0, 0]
		self.out_pitch = [0, 0, 0]
		self.out_throttle = [0, 0, 0]

		self.time_now = 0
		self.time_prev = 0
		self.time_change = 0


		# Hint : Add variables for storing previous errors in each axis, like self.prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		self.sample_time = 0.03333 # in seconds





		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', swift_msgs, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------
		
		self.altError = rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.pitchError = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.rollError = rospy.Publisher('/roll_error', Float64, queue_size=1)
		self.zero = rospy.Publisher('/zero', Int16, queue_size=1) #This is for displaying zero on plotjuggler
	#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		
		rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
		
		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE
	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)
	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)
	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x

		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z
		#---------------------------------------------------------------------------------------------------------------
	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.Ki * 0.00008
		self.Kd[2] = alt.Kd * 0.3
	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------
	def pitch_set_pid(self, pitch):
		self.Kp[1] = pitch.Kp * 0.06
		self.Ki[1] = pitch.Ki * 0.00008
		self.Kd[1] = pitch.Kd * 0.3

	def roll_set_pid(self, roll):
		self.Kp[0] = roll.Kp * 0.06
		self.Ki[0] = roll.Ki * 0.00008
		self.Kd[0] = roll.Kd * 0.3

	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum

		self.time_now = time.time()

		self.time_change = self.time_now - self.time_prev

		if self.time_change >= self.sample_time:
 
			for i in range(3):
				self.error[i] = self.drone_position[i] - self.setpoint[i]
				self.integral_error[i] += self.error[i]
				self.derivative_error[i] = (self.error[i] - self.prev_error[i])

			self.out_roll = (self.Kp[0]*self.error[0]) + (self.Ki[0]*self.integral_error[0]) + (self.Kd[0]*self.derivative_error[0])
			self.out_pitch = (self.Kp[1]*self.error[1]) + (self.Ki[1]*self.integral_error[1]) + (self.Kd[1]*self.derivative_error[1])
			self.out_throttle = (self.Kp[2]*self.error[2]) + (self.Ki[2]*self.integral_error[2]) + (self.Kd[2]*self.derivative_error[2])

			self.cmd.rcRoll = 1500 - round(self.out_roll)
			self.cmd.rcPitch = 1500 + round(self.out_pitch)
			self.cmd.rcThrottle = 1586 + round(self.out_throttle)

			if self.cmd.rcRoll > 2000:
				self.cmd.rcRoll = self.max_values[0]
			elif self.cmd.rcRoll < 1000:
				self.cmd.rcRoll = self.min_values[0]

			if self.cmd.rcPitch > 2000:
				self.cmd.rcPitch = self.max_values[1]
			elif self.cmd.rcPitch < 1000:
				self.cmd.rcPitch = self.min_values[1]

			if self.cmd.rcThrottle > 2000:
				self.cmd.rcThrottle = self.max_values[2]
			elif self.cmd.rcThrottle < 1000:
				self.cmd.rcThrottle = self.min_values[2]

			for i in range(3):
				self.prev_error[i] = self.error[i]


		self.time_prev = self.time_now








	#------------------------------------------------------------------------------------------------------------------------
		self.command_pub.publish(self.cmd)

		self.rollError.publish(self.error[0])
		self.pitchError.publish(self.error[1])
		self.altError.publish(self.error[2])

		self.zero.publish(0)
		
	def callback(self,data):
	
	# Used to convert between ROS and OpenCV images
		br = CvBridge()
		self.current_frame = br.imgmsg_to_cv2(data)

	def detector(self):
		self.led_no,avg_centroid=led_finder(self.current_frame)
		self.centroid=list(map(lambda x,y:((x-0.5)*24/2.6)+y,avg_centroid,self.drone_position))
		print(f'centroid	{self.centroid}')

	
	def landing(self):
		self.setpoint=[11,11,37]
		if max(list(map(abs,swift_drone.error)))<0.2:
			self.disarm()
	

if __name__ == '__main__':
	a = 24/6
	itr=0
	max_error=1
	points=[[0,0,25],
			[a,0,25],
			[2*a,0,25],
			[2*a,a,25],
			[2*a,2*a,25],
			[a,2*a,25],
			[a,a,25],
			[0,a,25],
			[0,2*a,25],
			[-a,2*a,25],
			[-2*a,2*a,25],
			[-2*a,a,25],
			[-a,a,25],
			[-a,0,25],
			[-2*a,0,25],
			[-2*a,-a,25],
			[-2*a,2*a,25],
			[-a,-2*a,25],
			[0,-2*a,25],
			[a,-2*a,25],
			[2*a,-2*a,25],
			[2*a,-a,25],
			[a,-a,25],
			[0,-a,25],
			[-a,-a,25],
			]
	swift_drone = swift()
	r = rospy.Rate(30) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	rospy.Subscriber('swift/camera_rgb/image_raw', Image, swift_drone.callback)
	while not rospy.is_shutdown():
		if max(list(map(abs,swift_drone.error)))<max_error:
			if(itr<len(points)):
				if(itr!=0):
					swift_drone.detector()
				if swift_drone.led_no==0:
					swift_drone.setpoint=points[itr]
					itr+=1
				else:
					swift_drone.setpoint=[swift_drone.centroid[0],swift_drone.centroid[1],25]
					max_error=0.2
		swift_drone.pid()
		r.sleep()