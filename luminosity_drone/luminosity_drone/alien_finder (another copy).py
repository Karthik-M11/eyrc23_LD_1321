#!/usr/bin/env python3

"""
Controller for the drone
"""

# standard imports
import copy
import time
import os

# third-party imports
import scipy.signal
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray, Vector3
from pid_msg.msg import PidTune
from swift_msgs.msg import PIDError, RCMessage
from swift_msgs.srv import CommandBool
from loc_msg.msg import Biolocation
# from led_detection import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import asyncio
import json
import sys
sys.path.insert(0, '~/colcon_ws/src/luminosity_drone/luminosity_drone/luminosity_drone')

MIN_ROLL = 1250
BASE_ROLL = 1500
MAX_ROLL = 1600
SUM_ERROR_ROLL_LIMIT = 10000

MAX_PITCH = 1600
BASE_PITCH = 1440
MIN_PITCH = 1250
SUM_ERROR_PITCH_LIMIT = 10000

MAX_THROTTLE = 1600
BASE_THROTTLE = 1460
MIN_THROTTLE = 1000
SUM_ERROR_THROTTLE_LIMIT = 20000

DRONE_WHYCON_POSE = [[], [], []]

file = open('arena_mapper.json')
data = json.load(file)
pid_itr=0
deci_pitch=BASE_PITCH

# Similarly, create upper and lower limits, base value, and max sum error values for roll and pitch
def point_setter(c_sp,next_sp,div):
    difference= list(map(lambda x,y: y -x, c_sp,next_sp))
    diff_norm=list(map(lambda x : x/div,difference))
    # print('Diff',diff_norm)
    new_sps=[]
    new_set_point=c_sp
    for i in range(div):
        new_set_point= list(map(lambda x,y: x + y, new_set_point,diff_norm))
        new_set_point=list(map(lambda x: round(x,3) , new_set_point))
        new_sps.append(new_set_point)
    return new_sps

def array_founder(arr,divisions=10):
    points=[arr[0]]
    for i in range(len(arr)-1):

        temp_pts=point_setter(arr[i],arr[i+1],divisions)
        for kuku in temp_pts:
            points.append(kuku)
        # print('Final: ',points)
    return points

class DroneController():
    def __init__(self,node :Node):
        self.node= node
        
        self.rc_message = RCMessage()
        self.drone_whycon_pose_array = PoseArray()
        self.last_whycon_pose_received_at = 0
        self.commandbool = CommandBool.Request()
        service_endpoint = "/swift/cmd/arming"

        self.drone_position = [-7.105, 6.998, 28.978]
        self.drone_orientation = [0,0,0,0]

        self.arming_service_client = self.node.create_client(CommandBool,service_endpoint)
        self.set_points = [0.097, -1.004, 25]        # Setpoints for x, y, z respectively      
        
        self.error = [0, 0, 0]         # Error for roll, pitch and throttle        

        # Create variables for integral and differential error
        self.integral_error = [0, 0, 0]
        self.derivative_error = [0, 0, 0]

        # Create variables for previous error and sum_error
        self.prev_error = [0, 0, 0]
        self.sum_error = [0, 0, 0]
        # rpy
        self.Kp = [ 180 * 0.01  , 220 * 0.01  , 300 * 0.01  ] #385 300 #260
 
        # Similarly create variables for Kd and Ki
        self.Ki = [ 100 * 0.0002  , 60 * 0.0002  , 350 * 0.0001  ] #300 
        self.Kd = [ 1400 * 0.1  , 1269 * 0.1  , 1538 * 0.1  ] #1538 #1750 #1200jfsdjg

        # Define variables to find number of led
        self.num_org = 0
        self.led_setpoint = [240, 320]
        self.led_error = [0, 0]

        # Variable to store current frame
        self.current_frame = cv2.imread("empty.png",1)

        self.led_roll = 0
        self.led_pitch = 0

        self.type_list = None
        self.cen_list = None
        self.centroid = None
        self.num_led = 0

        self.change_set = 0

        # self.itr99=0
        # self.prev99=0

        self.execution_time = 0

        # Bool to check if led has been identified in given video frame
        self.led_identified = False

        # Create subscriber for WhyCon 
        
        self.whycon_sub = node.create_subscription(PoseArray,"/whycon/poses",self.whycon_poses_callback,1)
        self.led_check_sub = node.create_subscription(Bool, "/led_check_pub", self.led_check_callback, 1)
        self.opencv_error = node.create_subscription(Vector3, "/opencv_error", self.opencv_error_callback, 1)
        
        # Similarly create subscribers for pid_tuning_altitude, pid_tuning_roll, pid_tuning_pitch and any other subscriber if required
       
        self.pid_alt = node.create_subscription(PidTune,"/pid_tuning_altitude",self.pid_tune_throttle_callback,1)
        self.pid_roll = node.create_subscription(PidTune, "/pid_tuning_pitch", self.pid_tune_roll_callback,1)
        self.pid_pitch = node.create_subscription(PidTune,"/pid_tuning_altitude",self.pid_tune_pitch_callback,1)

        # Subscriber for drone camera
        self.camera_sub = node.create_subscription(Image, "/video_frames", self.opencv_callback, 10)

        # Create publisher for sending commands to drone 

        self.rc_pub = node.create_publisher(RCMessage, "/swift/rc_command",1)

        # Create publisher for publishing errors for plotting in plotjuggler 
        
        self.pid_error_pub = node.create_publisher(PIDError, "/luminosity_drone/pid_error",1) 

        # Publisher to pubslish to astrobiolocation
        self.astrobiolocation = node.create_publisher(Biolocation, "/astrobiolocation", 1)

        # Timer to execute when the buzzer and led needs to be powered.
        self.on_bled = node.create_timer(0.2, self.on_callback)
        self.off_bled = node.create_timer(0.2, self.off_callback)
    
    def opencv_error_callback(self, msg):
        self.led_error[0] = msg.x
        self.led_error[1] = msg.y
        self.execution_time = msg.z * 2

    def led_check_callback(self, msg):
        self.led_identified = msg


    def whycon_poses_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z
        if(self.drone_position[0] != 1000):
            self.last_whycon_pose_received_at = self.node.get_clock().now().seconds_nanoseconds()[0]


    def pid_tune_throttle_callback(self, msg):
        self.Kp[2] = msg.kp * 0.01
        # self.Kp[2] = 480 *0.01
        self.Ki[2] = msg.ki * 0.0001
        self.Kd[2] = msg.kd * 0.1

    # Similarly add callbacks for other subscribers
    def pid_tune_roll_callback(self, msg):
        self.Kp[0] = msg.kp * 0.01
        self.Ki[0] = msg.ki * 0.0001
        self.Kd[0] = msg.kd * 0.1

    
    def pid_tune_pitch_callback(self, msg):
        self.Kp[1] = msg.kp * 0.01
        self.Ki[1] = msg.ki * 0.0001
        self.Kd[1] = msg.kd * 0.1


    def opencv_callback(self, data):
		# Used to convert between ROS and OpenCV images
        br = CvBridge()
		# Convert ROS Image message to OpenCV image
        self.current_frame = br.imgmsg_to_cv2(data)
        # cv2.imshow('vaaliban',self.current_frame)
        # cv2.waitKey(1)

    def led_detector(self, image):
        # self.type_list, self.cen_list, self.num_led = led_finder(image)
        # self.led_identified = len(self.type_list) > 0
        # self.centroid = self.drone_position
        # # self.num_led = len(self.cen_list)
        # print('Led_num',self.num_led)
        # # asyncio.run(self.b_led())
        pass


    def on_callback(self):
        #for buzzzer aux3, for led aux4
        if self.execution_time%2 == 0 and self.execution_time != 0:
            # print('on',self.execution_time)
            self.rc_message.aux3 = 2000
            self.rc_message.aux4 = 1500
            self.execution_time -= 1

    def off_callback(self):
        if self.execution_time%2 == 1:
            # print('off',self.execution_time)        self.set_points = [0.097, -1.004, 25]        # Setpoints for x, y, z respectively      

            self.rc_message.aux3 = 1000
            self.rc_message.aux4 = 2000
            self.execution_time -= 1

    # def b_led(self):
    #         self.itr99=0
    #         self.prev99=time.time()
    #         if((time.time()-self.prev99)>0.2):
    #             self.prev99=time.time()
    #             if(self.itr99%2==0):
    #                 self.rc_message.aux3 = 2000
    #                 self.rc_message.aux4 = 1500
    #                 self.rc_pub.publish(self.rc_message)
    #             else:
    #                 self.rc_message.aux3 = 1000
    #                 self.rc_message.aux4 = 2000
    #                 self.rc_pub.publish(self.rc_message)
    #             self.itr99 += 1

    # async def b_led(self):
    #     for i in range(self.num_led):
    #         self.rc_message.aux3 = 2000
    #         self.rc_message.aux4 = 1500
    #         self.rc_pub.publish(self.rc_message)
    #         await asyncio.sleep(1)
    #         self.rc_message.aux3 = 1000
    #         self.rc_message.aux4 = 2000
    #         self.rc_pub.publish(self.rc_message)
    #         await asyncio.sleep(1)
    
    def led_target(self):
        # self.type_list, cen_list, bye = led_finder(image)
        # # print(cen_list)
        # if(len(cen_list)!=0):
        #     self.led_error[0] = cen_list[0][0] - self.led_setpoint[0]
        #     self.led_error[1] = cen_list[0][1] - self.led_setpoint[1]
        #     self.led_roll = BASE_ROLL + round(0.1*self.led_error[0])
        #     self.led_pitch = BASE_PITCH - round(0.1*self.led_error[1])
        self.led_roll = BASE_ROLL + round(0.1*self.led_error[0])
        self.led_pitch = BASE_PITCH - round(0.1*self.led_error[1])	


    def pid(self):          # PID algorithm
        # print('in pid')[0.527, -0.718, 25] 
        # 0 : calculating Error, Derivative, Integral for Roll error : x axis
        global pid_itr,BASE_PITCH,deci_pitch
        
        try:
            self.error[0] = self.drone_position[0] - self.set_points[0] 
        # Similarly calculate error for y and z axes 
            self.error[1] = self.drone_position[1] - self.set_points[1]
            self.error[2] = self.drone_position[2] - self.set_points[2]
            # if max(list(map(abs,self.error)))<0.8:
            #     print(f"Time taken {Start-time.time()}")
        except IndexError:
            pass
        # print(self.error)
        # Calculate derivative and intergral errors. Apply anti windup on integral error (You can use your own method for anti windup, an example is shown here)
        # for i in range(2):
            # self.integral_error[i] += self.error[i]
        self.derivative_error[0] = (self.error[0] - self.prev_error[0])
        self.derivative_error[1] = (self.error[1] - self.prev_error[1])
        self.derivative_error[2] = (self.error[2] - self.prev_error[2])
        # self.integral_error[2] += self.error[2]
        self.integral_error[0] += self.error[0]
        self.integral_error[1] += self.error[1]
        self.integral_error[2] += self.error[2]
        # self.integral[0] = (self.integral[0] + self.error[0])
        # if self.integral[0] > SUM_ERROR_ROLL_LIMIT:

        #     self.integral[0] = SUM_ERROR_ROLL_LIMIT
        # if self.integral[0] < -SUM_ERROR_ROLL_LIMIT:
        #     self.integral[0] = -SUM_ERROR_ROLL_LIMIT
        if self.integral_error[0] > SUM_ERROR_ROLL_LIMIT:
            self.integral_error[0] = SUM_ERROR_ROLL_LIMIT
        if self.integral_error[0] < -SUM_ERROR_ROLL_LIMIT:
            self.integral_error[0] = -SUM_ERROR_ROLL_LIMIT
        if self.integral_error[1] > SUM_ERROR_PITCH_LIMIT:
            self.integral_error[1] = SUM_ERROR_PITCH_LIMIT
        if self.integral_error[1] < -SUM_ERROR_PITCH_LIMIT:
            self.integral_error[1] = -SUM_ERROR_PITCH_LIMIT
        if self.integral_error[2] > SUM_ERROR_THROTTLE_LIMIT:
            self.integral_error[2] = SUM_ERROR_THROTTLE_LIMIT
        if self.integral_error[2] < -SUM_ERROR_THROTTLE_LIMIT:
            self.integral_error[2] = -SUM_ERROR_THROTTLE_LIMIT

        # Save current error in previous error
        self.prev_error[0] = self.error[0]
        self.prev_error[1] = self.error[1]
        self.prev_error[2] = self.error[2]

        # 1 : calculating Error, Derivative, Integral for Pitch error : y axis
        self.out_roll = (self.Kp[0]*self.error[0]) + (self.Ki[0]*self.integral_error[0]) + (self.Kd[0]*self.derivative_error[0])
        self.out_pitch = (self.Kp[1]*self.error[1]) + (self.Ki[1]*self.integral_error[1]) + (self.Kd[1]*self.derivative_error[1])
        self.out_throttle = (self.Kp[2]*self.error[2]) + (self.Ki[2]*self.integral_error[2]) + (self.Kd[2]*self.derivative_error[2])

        # 2 : calculating Error, Derivative, Integral for Alt error : z axis


        # Write the PID equations and calculate the self.rc_message.rc_throttle, self.rc_message.rc_roll, self.rc_message.rc_pitch
        self.rc_message.rc_roll = BASE_ROLL + round(self.out_roll)
        self.rc_message.rc_pitch = BASE_PITCH - round(self.out_pitch)
        self.rc_message.rc_throttle = BASE_THROTTLE + round(self.out_throttle)

        if(pid_itr<=90):
            if(pid_itr==0):
                # print('Bp ',BASE_PITCH)
                self.divvu=(1500-BASE_PITCH)/90
                # print('ddivuu is ',self.divvu)
            deci_pitch+=self.divvu
            # print('deci pitch ',deci_pitch)
            BASE_PITCH=int(deci_pitch)
            pid_itr+=1
            print('Base Pitch is ',BASE_PITCH,' on itr ',pid_itr)
        
    #------------------------------------------------------------------------------------------------------------------------

        # check if led has been identifed or not and accordingly publish the params
        if self.led_identified == True:
            self.led_target()
            self.rc_message.rc_roll = self.led_roll
            self.rc_message.rc_pitch = self.led_pitch
            self.publish_data_to_rpi(roll=self.rc_message.rc_roll, pitch=self.rc_message.rc_pitch, throttle=self.rc_message.rc_throttle)

        else:
        # self.publish_data_to_rpi( roll = 1500, pitch = 1500, throttle = 1000)
            self.publish_data_to_rpi(roll=self.rc_message.rc_roll, pitch=self.rc_message.rc_pitch, throttle=self.rc_message.rc_throttle)

        #Replace the roll pitch and throttle values as calculated by PID 
        
        
        # Publish alt error, roll error, pitch error for plotjuggler debugging

        self.pid_error_pub.publish(
            PIDError(
                roll_error=float(self.error[0]),
                pitch_error=float(self.error[1]),
                throttle_error=float(self.error[2]),
                yaw_error=-0.0,                # Setpoints for x, y, z respectively      
                zero_error=0.0,
            )
        )


    def publish_data_to_rpi(self, roll, pitch, throttle):

        self.rc_message.rc_throttle = int(throttle)
        self.rc_message.rc_roll = int(roll)
        self.rc_message.rc_pitch = int(pitch)

        # Send constant 1500 to rc_message.rc_yaw
        self.rc_message.rc_yaw = int(1500)

        # BUTTERWORTH FILTER
        span = 15
        for index, val in enumerate([roll, pitch, throttle]):
            DRONE_WHYCON_POSE[index].append(val)
            if len(DRONE_WHYCON_POSE[index]) == span:
                DRONE_WHYCON_POSE[index].pop(0)
            if len(DRONE_WHYCON_POSE[index]) != span-1:
                return
            order = 3
            fs = 60
            fc = 5
            nyq = 0.5 * fs
            wc = fc / nyq
            b, a = scipy.signal.butter(N=order, Wn=wc, btype='lowpass', analog=False, output='ba')
            filtered_signal = scipy.signal.lfilter(b, a, DRONE_WHYCON_POSE[index])
            if index == 0:
                self.rc_message.rc_roll = int(filtered_signal[-1])
            elif index == 1:
                self.rc_message.rc_pitch = int(filtered_signal[-1])
            elif index == 2:
                self.rc_message.rc_throttle = int(filtered_signal[-1])

        if self.rc_message.rc_roll > MAX_ROLL:     #checking range i.e. bet 1000 and 2000
            self.rc_message.rc_roll = MAX_ROLL
        elif self.rc_message.rc_roll < MIN_ROLL:
            self.rc_message.rc_roll = MIN_ROLL

        # Similarly add bounds for pitch yaw and throttle 
        if self.rc_message.rc_pitch > MAX_PITCH:
            self.rc_message.rc_pitch = MAX_PITCH
        elif self.rc_message.rc_pitch < MIN_PITCH:
            self.rc_message.rc_pitch = MIN_PITCH
        
        if self.rc_message.rc_throttle > MAX_THROTTLE:
            self.rc_message.rc_throttle = MAX_THROTTLE
        elif self.rc_message.rc_throttle < MIN_THROTTLE:
            self.rc_message.rc_throttle = MIN_THROTTLE
        

        # Similarly add bounds for pitch yaw and throttle 

        self.rc_pub.publish(self.rc_message)

    # This function will be called as soon as this rosnode is terminated. So we disarm the drone as soon as we press CTRL + C. 
    # If anything goes wrong with the drone, immediately press CTRL + C so that the drone disamrs and motors stop 

    def shutdown_hook(self):
        self.node.get_logger().info("Calling shutdown hook")
        self.disarm()

    # Function to arm the drone 

    def arm(self):
        self.node.get_logger().info("Calling arm service")
        self.commandbool.value = True
        self.future = self.arming_service_client.call_async(self.commandbool)

    # Function to disarm the drone 

    def disarm(self):
        while True:
        # Create the disarm function
            self.node.get_logger().info("Calling disarm service")
            self.commandbool.value = False
            self.future = self.arming_service_client.call_async(self.commandbool)


def main(args=None):
    rclpy.init(args=args)
    itr=0
    landing_itr = 0
    publish_check = False
    check_set = 0
    currentTime = 0
    prevTime = 0
    location = Biolocation()
    # State 0 implies mapping, state 1 implies moving towards led, state 2 implies landing
    state = 0 
    points = []
    for i in data:
        data[i][2] = 25
    points.append(data['E5'])
    points.append(data['D5'])
    points.append(data['C5'])
    points.append(data['B5'])
    points.append(data['A5'])
    points.append(data['A4'])
    points.append(data['A3'])
    points.append(data['A2'])
    points.append(data['A1'])
    points.append(data['B1'])
    points.append(data['C1'])
    points.append(data['D1'])
    points.append(data['E1'])
    points.append(data['E2'])
    points.append(data['E3'])
    points.append(data['E4'])
    points.append(data['D4'])
    points.append(data['C4'])
    points.append(data['B4'])
    points.append(data['B3'])
    points.append(data['B2'])
    points.append(data['C2'])
    points.append(data['D2'])
    points.append(data['D3'])
    points.append(data['C3'])

    points.reverse()
    # print('old',points)
    points=array_founder(points,3)
    print('new',points)
    
    # points.append(data['C3'])
    # points.append(data['D4'])
    # points.append(data['B4'])
    # points.append(data['B2'])
    # points.append(data['D2'])
    # points.append(data['A4'])

    file.close()
    
    landing = [points[-1],[points[-1][0],points[-1][1], 28.5]]

    node = rclpy.create_node('controller')
    node.get_logger().info(f"Node Started")
    node.get_logger().info("Entering PID controller loop")

    controller = DroneController(node)
    controller.arm()
    node.get_logger().info("Armed")
    try:
        while rclpy.ok():
            # if controller.current_frame != None:
            # print(controller.current_frame.shape)
            if controller.num_org == 1 and state!=2:
                state = 2
                print('To Landing')
            if state == 0:
                # print(controller.num_org)
                if max(list(map(abs,controller.error)))<0.8:
                    if check_set == 0:
                        prevTime = time.time()
                        check_set = 1
                    currentTime = time.time()
                    # check_set += 1
                    if itr == len(points):
                        state = 2
                        print('To Landing')

                    elif(itr<len(points)):
                        # if itr!=0:
                            # print('check1')
                            # controller.led_detector(controller.current_frame)
                            # if controller.led_identified == True:
                            #     state = 1
                            #     print('Moving to led')
                        # if check_set % 30 == 0:
                        
                        if currentTime - prevTime > 0.5:
                            # controller.led_detector(controller.current_frame)
                            if controller.led_identified == True:
                                state = 1
                                print('Moving to led')
                            controller.set_points=points[itr]
                            prevTime = currentTime = 0
                            check_set = 0
                            itr += 1
                            
                        # check_set = 0                    
                        # controller.integral_error = [0, 0, 0]
                        # itr+=1
                            print(controller.set_points)
                    

            elif state == 1:
                if controller.led_error != [0, 0] and max([abs(i) for i in controller.led_error]) < 20:
                    print('Aligned')
                    # controller.led_detector(controller.current_frame)
                    # controller.execution_time = controller.num_led * 2
                    # print(controller.execution_time)
                    # publish_check = True
                    # controller.led_identified == False
                    state = 0
                    controller.num_org += 1
            
            elif state == 2:
                if landing_itr<2:
                    controller.set_points = landing[landing_itr]
                if landing_itr == 2:
                    print('Landed')
                    controller.shutdown_hook()
                    node.destroy_node()
                    rclpy.shutdown()
                elif max(list(map(abs,controller.error)))<0.8:
                    landing_itr+=1

            # if publish_check == True:
            #     print('Published')
            #     controller.num_org += 1
            #     location.organism_type = controller.type_list[0]
            #     location.whycon_x = controller.centroid[0]
            #     location.whycon_y = controller.centroid[1]
            #     location.whycon_z = controller.centroid[2]
            #     controller.astrobiolocation.publish(location)
            #     publish_check = False

            controller.pid()
            rclpy.spin_once(node) # Sleep for 1/30 secs
            if controller.drone_position == None:
                node.get_logger().error("Unable to detect WHYCON poses")
                controller.shutdown_hook()
                node.destroy_node()
                rclpy.shutdown()
        

    except Exception as err:
        print(err)


    finally:
        controller.shutdown_hook()
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()