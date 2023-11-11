#!/usr/bin/env python3

'''
# Team ID:           1321
# Theme:             Luminosity Drone
# Author List:       M Krishnaprasad Varma, Karthik Manoranjan, Madhav Menon, Sneha Joe M
# Filename:          biota_detector.py
# Functions:         __init__ , disarm , arm , whycon_callback , altitude_set_pid , pitch_set_pid , roll_set_pid , led_detector , led_target , led_finder , opencv_callback , identify , pid , literal_eval
# Global variables:  None
'''


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
import rospy
import time
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from led_detection import led_finder
from luminosity_drone.msg import Biolocation
import ast


class swift():
	"""docstring for swift"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [0,0,30] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly


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


		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters

		self.Kp = [16, 18, 28.8] #[22.5, 22.5, 28.8]   #[22.5, 0, 40.8]
		self.Ki = [-0.007, -0.007, -0.01] #[0, 0, 0.174]   #[0, 0, 0.18575]
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

		self.num_led = 0
		self.centroid = None
		self.current_frame = None

		self.cen_error_check = False
		self.led_setpoint = [0.5, 0.5]
		self.led_error = [0,0]
		self.led_check = 0

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
		self.astrobiolocation = rospy.Publisher('/astrobiolocation', Biolocation, queue_size=1)
	#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		
		rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
		rospy.Subscriber('/swift/camera_rgb/image_raw', Image, self.opencv_callback)
		
		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE

	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 0
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


	def led_detector(self, image):
		'''
		Purpose:
		---
		To detect the number of LEDs present in the input frame and updates the centroid value with the drone position when called.

		Input Arguments:
		---
		`image` :  [ ndarray ]
			The image to be analysed.

		Returns:
		---
		None

		Example call:
		---
		led_detector(image)
		'''
		area, centroid = led_finder(image)
		self.num_led = len(area)
		self.centroid = self.drone_position


	def led_target(self, image):
		'''
		Purpose:
		---
		To identify the normalised centroid of the LEDs and align the centre of drone camera output and the LED centroid by a proportional controller.

		Input Arguments:
		---
		`image` :  [ ndarray ]
			The image to be analysed.

		Returns:
		---
		None

		Example call:
		---
		led_target(image)
		'''
		area, target = led_finder(image)
		self.led_error[0] = target[0] - self.led_setpoint[0]
		self.led_error[1] = target[1] - self.led_setpoint[1]
		roll = 30*self.led_error[0]
		pitch = 30*self.led_error[1]
		self.cmd.rcRoll = 1500 + round(roll)
		self.cmd.rcPitch = 1500 - round(pitch)	
		self.led_check = len(area)


	# OpenCV callback function.
	# Executes every time data is published to /swift/camera_rgb/image_raw 
	def opencv_callback(self, data):
		# Used to convert between ROS and OpenCV images
		br = CvBridge()
		# Convert ROS Image message to OpenCV image
		self.current_frame = br.imgmsg_to_cv2(data)


	def identify(self):
		'''
		Purpose:
		---
		To identify the type of organism based on the number of LEDs detected.

		Input Arguments:
		---
		None

		Returns:
		---
		`org_type` :  [ string ]
    		Type of organism.

		Example call:
		---
		identify()
		'''
		if self.num_led == 2:
			org_type = "alien_a"
		elif self.num_led == 3:
			org_type = "alien_b"
		elif self.num_led == 4:
			org_type = "alien_c"
		else:
			org_type = "human"

		return org_type

	#----------------------------------------------------------------------------------------------------------------------

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

			if self.num_led == 0:
				self.out_roll = (self.Kp[0]*self.error[0]) + (self.Ki[0]*self.integral_error[0]) + (self.Kd[0]*self.derivative_error[0])
				self.out_pitch = (self.Kp[1]*self.error[1]) + (self.Ki[1]*self.integral_error[1]) + (self.Kd[1]*self.derivative_error[1])
				self.cmd.rcRoll = 1500 - round(self.out_roll)
				self.cmd.rcPitch = 1500 + round(self.out_pitch)
			else:
				self.led_target(self.current_frame)
			self.out_throttle = (self.Kp[2]*self.error[2]) + (self.Ki[2]*self.integral_error[2]) + (self.Kd[2]*self.derivative_error[2])
			

			# self.cmd.rcRoll = 1500 - round(self.out_roll)
			# self.cmd.rcPitch = 1500 + round(self.out_pitch)
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
		







if __name__ == '__main__':
	itr = 0
	step = 0
	led_detected = 0
	
	# Update the path with the path to setpoints.txt
	with open(r'/home/karthik/eyantra_ws/src/luminosity_drone/luminosity_drone/scripts/setpoints.txt', 'r') as file:
		content = file.read()
	points = ast.literal_eval(content)

	swift_drone = swift()
	location = Biolocation()
	r = rospy.Rate(30) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz	
	while not rospy.is_shutdown():
		# Checks if drone position is within applicable limits when the setpoint is present in the list of points.
		if max(list(map(abs,swift_drone.error[0:2])))<0.4 and swift_drone.error[2] < 1 and swift_drone.setpoint in points and step != 1:
			if itr<len(points):
				if itr!=0:
					swift_drone.led_detector(swift_drone.current_frame)
				swift_drone.setpoint = points[itr]
				swift_drone.integral_error = [0, 0, 0]
				itr+=1

		# Aligns the centre of drone camera frame with the centroid of the LEDs within the acceptable tolerance.
		if swift_drone.num_led > 0 and swift_drone.led_error != [0,0]:
			step = 1
			print([abs(i) for i in swift_drone.led_error])
			if max([abs(i) for i in swift_drone.led_error]) < 0.02:
				swift_drone.cen_error_check = True
				swift_drone.led_detector(swift_drone.current_frame)
				if swift_drone.led_check == 0:
					swift_drone.num_led = 0
					swift_drone.setpoint = points[itr]			

		# Publishes to /astrobiolocation
		if swift_drone.cen_error_check == True:
			location.organism_type=swift_drone.identify()
			location.whycon_x=swift_drone.centroid[0]
			location.whycon_y=swift_drone.centroid[1]
			location.whycon_z=swift_drone.centroid[2]
			swift_drone.astrobiolocation.publish(location)
			swift_drone.num_led = 0
			swift_drone.setpoint = points[itr]
			swift_drone.integral_error = [0,0,0]
			step = 0
			swift_drone.cen_error_check = False
			swift_drone.led_error = [0,0]
			led_detected += 1

		# Drone goes to landing if either the drone covers all setpoints or 3 organisms are detected.
		if itr == len(points) or led_detected == 3:
			swift_drone.setpoint = [10.2, 10.4, 37.5]
			swift_drone.Kp = [12, 20, 28.8]
			swift_drone.integral_error = [0,0,0]
			if abs(swift_drone.drone_position[0] - 11) < 0.1 and abs(swift_drone.drone_position[1] - 11) < 0.1:
				swift_drone.disarm()
				step = 3

		if step != 3:
			swift_drone.pid()
			r.sleep()