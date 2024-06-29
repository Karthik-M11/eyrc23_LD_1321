#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int16, Bool
from cv_bridge import CvBridge
import cv2
import cv2, socket, numpy, pickle
import os
from loc_msg.msg import Biolocation
from geometry_msgs.msg import PoseArray, Vector3

class VideoPublisherNode():
    def __init__(self,node :Node):

        self.node= node
        self.drone_position = [0, 0, 0]
        # Define variables to find number of led
        self.num_org = 0
        self.led_setpoint = [240, 320]
        self.led_error = [0.0, 0.0]

        # Variable to store current frame
        self.current_frame = cv2.imread("empty.png",1)

        self.led_roll = 0
        self.led_pitch = 0

        self.type_list = None
        self.cen_list = None
        self.centroid = None
        self.num_led = 0.0


        self.execution_time = 0

        # Bool to check if led has been identified in given video frame
        self.led_identified = False

        self.subscriber = self.node.create_subscription(Image, '/video_frames', self.opencv_callback, 10)
        self.led_check_pub = self.node.create_publisher(Bool, '/led_check_pub', 1)
        self.opencv_error = self.node.create_publisher(Vector3, "/opencv_error", 1)
        self.whycon_sub = self.node.create_subscription(PoseArray,"/whycon/poses",self.whycon_poses_callback,1)
        self.astrobiolocation = self.node.create_publisher(Biolocation, "/astrobiolocation", 1)

        self.bridge = CvBridge()

    def opencv_callback(self, data):
		# Used to convert between ROS and OpenCV images
        br = CvBridge()
		# Convert ROS Image message to OpenCV image
        self.current_frame = br.imgmsg_to_cv2(data)
        # cv2.imshow('vaaliban',self.current_frame)
        # cv2.waitKey(1)


    def check_distance(self, coord_1, coord_2, cutoff):        
        # Calculates the euclidean distance between the two input coordinats and compared them against the given cutoff.
        if cutoff**2 > (coord_1[0]-coord_2[0])**2 + (coord_1[1]-coord_2[1])**2:
            return True
        else:
            return False
        
    def clusterize_led(self, circles, cutoff):    
        list_1 = []
        list_2 = []
        ran_val = 0

        list_1.append(circles[0])

        for i in circles:
            if ran_val == 0:
                ran_val += 1
                continue
            else:
                check = self.check_distance(i, list_1[0], cutoff)
                if check == True:
                    list_1.append(i)
                else:
                    list_2.append(i)

        return [list_1, list_2]
        

    def identify(self, num_led):
        # print('Led number ',num_led)
        if num_led == 2:
            org_type = "alien_a"
        elif num_led == 3:
            org_type = "alien_b"
        elif num_led == 4:
            org_type = "alien_c"
        elif num_led == 5:
            org_type = "alien_d"
        else:
            org_type = "human"

        return org_type
    
    
    def calculate_avg_centroid(self, organism):      
        sum_x = 0
        sum_y = 0
        length = len(organism)
        for i in organism:
            sum_x += i[0]
            sum_y += i[1]
        
        avg_x = sum_x/length
        avg_y = sum_y/length
        return [avg_x, avg_y]
    
    def whycon_poses_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z
    
    def led_detector(self, image):
        self.type_list, self.cen_list, self.num_led = self.led_finder(image)
        self.led_identified = len(self.type_list) > 0
        # self.led_check_pub.publish(True)
        self.centroid = self.drone_position
        # self.num_led = len(self.cen_list)
        # print('Led_num',self.num_led)
        # asyncio.run(self.b_led())

    def led_target(self, image):
        self.type_list, cen_list, bye = self.led_finder(image)
        # print(cen_list)
        if(len(cen_list)!=0):
            self.led_error[0] = cen_list[0][0] - self.led_setpoint[0]
            self.led_error[1] = cen_list[0][1] - self.led_setpoint[1]

    def led_finder(self, image):
        # dimension = image.shape    

        # convert it to grayscale, and blur it
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur_image = cv2.blur(gray_image, (11,11))

        # threshold the image to reveal light regions in the blurred image
        threshold_image = cv2.threshold(blur_image, 90, 255, cv2.THRESH_BINARY)[1]

        # perform a series of erosions and dilations to remove any small blobs of noise from the thresholded image
        threshold_image = cv2.erode(threshold_image, None, iterations=2)
        threshold_image = cv2.dilate(threshold_image, None, iterations=3)

        # Using Hough circles transform to identify the circles in the given image
        circles = cv2.HoughCircles(threshold_image, cv2.HOUGH_GRADIENT, 1, 20,
                                param1=50, param2=15,
                                minRadius=0, maxRadius=0)
        
        # Executed if no circles are identified in the given image
        if circles is None:
            return [] , [], 0
        # Executed if circles are identified in the given image
        else:
            cutoff = 10*circles[0][0][2] # The cutoff distance is 10 times the radius of the first led identified

            # To draw the circle outline
            for i in circles[0, :]:
                center = (int(i[0]), int(i[1]))
                radius = int(i[2]) - 2
                cv2.circle(image, center, radius, (0, 0, 255), 3)

            org_list = []
            input_list = circles[0] # List input to be given to clusterize_led function
            check = False

            # The while loop will execute as long as the second list returned by the clusterize_led function has elements 
            while check == False:
                temp_list1, temp_list2 = self.clusterize_led(input_list, cutoff)
                org_list.append(temp_list1)
                if len(temp_list2) == 0:
                    check = True
                else:
                    input_list = temp_list2

            cen_list = []
            type_list = []
            num_of_led = 0
            for i in org_list:
                # Code to calculate the centroid of the identified organisms and append them to a list.
                avg_cen = self.calculate_avg_centroid(i)
                # cv2.putText(image, f"{len(i)}", (int(avg_cen[0]), int(avg_cen[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 2)
                cen_list.append(avg_cen)

                # Code to identify the type of the identified organisms and append them to a list.
                num_of_led=len(i)
                org_type = self.identify(num_of_led)
                type_list.append(org_type)   

            # cv2.imshow(f"{image_name}_result", image)    
            # cv2.imwrite(f"{image_name}_result.png", image)
            # cv2.waitKey(0)     

            return type_list, cen_list ,num_of_led

def main(args=None):
    rclpy.init(args=args)
    video = rclpy.create_node('node')
    data = Vector3()
    location = Biolocation()
    node = VideoPublisherNode(video)
    publish_check = False
    # print('hi')
    try:
        while rclpy.ok():  
            node.led_detector(node.current_frame)
            data.x = node.led_error[0]
            data.y = node.led_error[1]
            data.z = float(node.num_led)
            node.opencv_error.publish(data)
            print('hi')

            if node.led_error != [0, 0] and max([abs(i) for i in node.led_error]) < 20:
                print('Aligned')
                # controller.led_detector(controller.current_frame)
                # print(controller.execution_time)
                publish_check = True
                node.led_identified == False
            
            if publish_check == True:
                print('Published')
                location.organism_type = node.type_list[0]
                location.whycon_x = node.centroid[0]
                location.whycon_y = node.centroid[1]
                location.whycon_z = node.centroid[2]
                node.astrobiolocation.publish(location)
                publish_check = False
            rclpy.spin(video)
    except KeyboardInterrupt:
        video.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
