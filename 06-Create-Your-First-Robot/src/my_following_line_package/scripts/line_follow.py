#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

class LineFollower(object):

    def __init__(self):
    
        self.bridge_object = CvBridge()
    
    # Convert to numpy array and return converted data
    def convert_image(self, data):
        im = np.frombuffer(data.data, dtype=np.uint8).reshape(data.width, data.height, -1)
        return im

    def process_data(self,observation_image, detection_image, rows_to_watch ,show_window=False):
        recalculate = False
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(detection_image, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            
        # We get image dimensions and crop the parts of the image we don't need
        # Bear in mind that because the first value of the image matrix is start and second value is down limit.
        # Select the limits so that it gets the line not too close and not too far, and the minimum portion possible
        # To make process faster.
        height, width, channels = cv_image.shape
        top_trunc = 1*height / 2 #get 1/2 of the height from the top section of the image
        bottom_trunc = top_trunc + rows_to_watch #next set of rows to be used
        crop_img = cv_image[top_trunc:bottom_trunc, 0:width]
        
        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV).astype(np.float)
        
        # Define the Yellow Colour in HSV
        #RGB
        #[[[222,255,0]]]
        #BGR
        #[[[0,255,222]]]
        """
        To know which color to track in HSV, Put in BGR. Use ColorZilla to get the color registered by the camera
        >>> yellow = np.uint8([[[B,G,R ]]])
        >>> hsv_yellow = cv2.cvtColor(yellow,cv2.COLOR_BGR2HSV)
        >>> print( hsv_yellow )
        [[[ 34 255 255]]
        """
        lower_yellow = np.array([20,100,100])
        upper_yellow = np.array([50,255,255])

        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2
            recalculate = True
        
        if(show_window):
            # Bitwise-AND mask and original image
            res = cv2.bitwise_and(crop_img,crop_img, mask= mask)
            
            # Draw the centroid in the resultut image
            # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
            cv2.circle(res,(int(cx), int(cy)), 10,(0,0,255),-1)
    
            cv2.imshow("Original", cv_image)
            cv2.imshow("HSV", hsv)
            cv2.imshow("MASK", mask)
            cv2.imshow("RES", res)
            
            cv2.waitKey(1)
        
        error_x = cx - width / 2;
        angular_z = -error_x / 100;
        rospy.loginfo("ANGULAR VALUE SET===>"+str(angular_z))
        return (recalculate, angular_z, self.convert_image(observation_image))
        
    def clean_up(self):
        cv2.destroyAllWindows()
        