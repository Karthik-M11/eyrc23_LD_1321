'''
# Team ID:           1321
# Theme:             Luminosity Drone
# Author List:       M Krishnaprasad Varma, Karthik Manoranjan, Madhav Menon, Sneha Joe M
# Filename:          led_detection.py
# Functions:         clusterize_led, check_distance, identify, calculate_avg_centroid, led_finder
# Global variables:  None
'''

# import the necessary packages
from imutils import contours
from skimage import measure
import numpy as np
import imutils
import cv2
import argparse


# Code to create a parser argument to input the image file name
parser = argparse.ArgumentParser(description='Obtain image path')
parser.add_argument('--image', type=str, help='name of the image file')
args = parser.parse_args()


def clusterize_led(circles, cutoff):
    '''Purpose:
		---
		To group the given set of coordinates based on a given cutoff distance.

		Input Arguments:
		---
		'circles' : [ list ]
            Nested list of coordinates to be grouped.
        
        'cutoff' : [ float ]
            Cutoff distance to be used for grouping given set of coordinates.

		Returns:
		---
		`grouped coordinates` :  [ list ]
    		Nested list containing two groups of coordinates.

		Example call:
		---
		clusterize_led([[1,1,1], [2,2,2], [3,4,5]], 2)'''
    
    list_1 = []
    list_2 = []
    ran_val = 0

    list_1.append(circles[0])

    for i in circles:
        if ran_val == 0:
            ran_val += 1
            continue
        else:
            check = check_distance(i, list_1[0], cutoff)
            if check == True:
                list_1.append(i)
            else:
                list_2.append(i)

    return [list_1, list_2]


def check_distance(coord_1, coord_2, cutoff):
    '''Purpose:
		---
		To calculate the distance between two coordinates and compare it with given cutoff distance.

		Input Arguments:
		---
		'coord_1' : [ list ]
            Coordinates of the first organism.

        'coord_2' : [ list ]
            Coordinates of the second organism.

        'cutoff' : [ float ]
            Cutoff distance against which calculated distance need to be compared with.

		Returns:
		---
		`check_value` :  [ bool ]
    		True or False.

		Example call:
		---
		check_distance([1,2,3], [2,3,4], 5)'''
    
    # Calculates the euclidean distance between the two input coordinats and compared them against the given cutoff.
    if cutoff**2 > (coord_1[0]-coord_2[0])**2 + (coord_1[1]-coord_2[1])**2:
        return True
    else:
        return False
    

def identify(num_led):
    '''Purpose:
		---
		To identify the type of organism based on the number of LEDs detected.

		Input Arguments:
		---
		'num_led' : [ int ]
            Number of leds in the identified organism.

		Returns:
		---
		`org_type` :  [ string ]
    		Type of organism.

		Example call:
		---
		identify(2)'''
    
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
 

def calculate_avg_centroid(organism):
    '''Purpose:
		---
		To calculate the average centroid of the identified organism.

		Input Arguments:
		---
		'organism' : [ list ]
            List of centroids of the leds in the identified organism.

		Returns:
		---
		`centroid` :  [ list ]
    		Centroid of the organism

		Example call:
		---
		calculate_avg_centroid([1,1,1], [2,2,2], [3,3,3])'''
    
    sum_x = 0
    sum_y = 0
    length = len(organism)
    for i in organism:
        sum_x += i[0]
        sum_y += i[1]
    
    avg_x = sum_x/length
    avg_y = sum_y/length
    return [avg_x, avg_y]


def led_finder(image, image_name):
    '''
    Purpose:
    ---
    To detect the organisms present in the given image and show the number of leds present in each organism.

    Input Arguments:
    ---
    `image` :  [ ndarray ]
        The image to be analysed.
    
    'image_name' : [ string ]
        The name of the image file to be analysed.

    Returns:
    ---
    `type_list` :  [ list ]
        List of type of each detected organism.

    `cen_list` :  [ list ]
        List of centroid of each detected organism.

    Example call:
    ---
    led_finder(image.png)
    '''
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
        return [] , []
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
            temp_list1, temp_list2 = clusterize_led(input_list, cutoff)
            org_list.append(temp_list1)
            if len(temp_list2) == 0:
                check = True
            else:
                input_list = temp_list2

        cen_list = []
        type_list = []
        for i in org_list:
            # Code to calculate the centroid of the identified organisms and append them to a list.
            avg_cen = calculate_avg_centroid(i)
            cv2.putText(image, f"{len(i)}", (int(avg_cen[0]), int(avg_cen[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 2)
            cen_list.append(avg_cen)

            # Code to identify the type of the identified organisms and append them to a list.
            org_type = identify(len(i))
            type_list.append(org_type)   

        cv2.imshow(f"{image_name}_result", image)    
        cv2.imwrite(f"{image_name}_result.png", image)
        cv2.waitKey(0)     

        return type_list, cen_list


image_png = args.image # Obtains the image file name from the argument parser
image = cv2.imread(image_png, 1) # Reads the image
image_name = image_png.split('.')[0]
type_list, cen_list = led_finder(image, image_name)

# Code to create file image_name.txt corresponding to the image_name.png file.
with open(f"{image_name}.txt", "w") as file:
        for i in range(len(type_list)):
            file.write(f"Organism Type: {type_list[i]}\n")
            file.write(f"Centroid: {cen_list[i]}\n\n")
file.close()
