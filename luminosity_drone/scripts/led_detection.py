# import the necessary packages
from imutils import contours
from skimage import measure
import numpy as np
import imutils
import cv2
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library

# load the image, 
def led_finder(image):
    # image = cv2.imread('led.jpg', 1)
    # cv2.imshow('image', image)
    # cv2.waitKey(0)

    # convert it to grayscale, and blur it
    image=cv2.rotate(image,cv2.ROTATE_180)
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
          return 0
    else:
        contour = contours.sort_contours(contour)[0]
    # loop over the contours
    # Initialize lists to store centroid coordinates and area
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

    return len(Area)
    # Save the output image as a PNG file
    # cv2.imwrite("LD_1321_led_detection_results.png", image)
    # Open a text file for writing
    # with open("LD_1321_led_detection_results.txt", "w") as file:
    #     # Write the number of LEDs detected to the file
    #         file.write(f"No. of LEDs detected: {len(Area)}\n")
    #     # Loop over the contoursf
    #         for i in range(len(Area)):
    #         # Write centroid coordinates and area for each LED to the file
    #                 file.write(f"Centroid #{i + 1}: {Centroid[i]}\nArea #{i + 1}: {Area[i]}\n")
    # # Close the text file
    # file.close()
