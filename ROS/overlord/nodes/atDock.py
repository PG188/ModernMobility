#data_out = list(pc2.read_points(kinect_depth, field_names=("x", "y", "z"), skip_nans=True, uvs=[[1, 1], [600,1], [600, 400], [1, 400]]))

#!/usr/bin/env python

#import sys
#sys.path.append('atDock_Dependencies')

import cv2
import numpy as np
import time
import math
import TriangulatePosition as tp
import getLR
import sensor_msgs.point_cloud2 as pc2

IRL_SYM_WIDTH = 0.2032 #meters (0.2032m = 8 inches)

class Pose():
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

def avgPoints(single_contour, number_of_points):
    x = 0
    y = 0
    
    for point in single_contour:
        x += point[0][0]
        y += point[0][1]

    x /= number_of_points
    y /= number_of_points

    return x, y

def getDists(left_coord, right_coord, kinect_depth):
    """
    coords look like = [x, z]
    """
    depth_data = list(pc2.read_points(kinect_depth, field_names=("x", "y", "z"), skip_nans=False, uvs=[left_coord, right_coord]))    #[[(x,y,z)]]
    print('(x, y, z) from depth: %s' %(depth_data))
    return depth_data[0][2], depth_data[1][2]

def atDock(kinect_image, kinect_depth):
    print ("[atDock.py]: Starting the atDock() function...")

    #Constants
    FONT = cv2.FONT_HERSHEY_SIMPLEX #font type
    E_COEFF = 0.01   #Coefficient for epsilon value
    SHAPE_CORNERS = 12
    BLUE = (255,0,0)
    GREEN = (0,255,0)
    RED = (0,0,255)

    #Variables
    possible_contours = []
                
    try:
        #_, inFrame = cap.read() #get the video frame
        outFrame = cv2.cvtColor(kinect_image, cv2.COLOR_BGR2GRAY)  #converts to grayscale
        #outFrame = cv2.fastNlMeansDenoising(outFrame,None,50,7,21) #filters out noise (frame, None, higher filter out more noise but gives less detail, filter parameter, filtaer parameter)
        _, mask = cv2.threshold(outFrame, 50, 255, cv2.THRESH_BINARY_INV)    #If pixel value is above 220 it will convert to 255(white), below will turn to black (because binary)
        outFrame = cv2.bitwise_not(mask)
        outFrame = cv2.Canny(outFrame, 50, 50)

        _, contours, _ = cv2.findContours(outFrame,1,2)

        print ("[atDock.py]: Looking for Contours in image...")
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt,E_COEFF*cv2.arcLength(cnt,True),True)
            length = len(approx)
            
            #Record any contours with the desired number of points
            if length == SHAPE_CORNERS:
                possible_contours.append(cnt)
                
    except Exception as e:
        print("[atDock.py]: ERROR OCCURED WHILE ANALYSING THE IMAGE:\n\t" + str(e))
        
    #If no contours are found return None type object
    if(len(possible_contours) <= 0):
        #cv2.imshow('kinect_image', kinect_image)
        #cv2.imshow('outFrame', outFrame)
        print ("[atDock()]: Not %s-point contours found! Returned None." % SHAPE_CORNERS)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return None
    
    #Else proceed as planned
    else:
        try:
            #Find the largest contour and display it
            largest_contour = max(possible_contours, key = cv2.contourArea)
            
            #cv2.drawContours(kinect_image, [largest_contour], 0, RED, -1)
            #cv2.imshow("kinect_image", kinect_image)
            print ("[atDock.py]: Found a contour! Start processing...")

            #Get the center of the contour
            symX, symZ = avgPoints(largest_contour, SHAPE_CORNERS)           

            #Get the center of the kinect image frame
            rows, cols, _ = kinect_image.shape
            camX = int(cols/2)
            camZ = int(rows/2)

        #Find the position vector to the center of the symbol
            print ("[atDock.py]: Calling getLR.getLR()...\n")
            leftPoint, rightPoint = getLR.getLR(largest_contour)
            print ("\n[atDock.py]: ...getLR.getLR() finished executing")
            
            #Kinect origin at bottom-left, OpenCV origin at top-left
            lpx = leftPoint[0]
            lpz = rows - leftPoint[1]   
            rpx = rightPoint[0]
            rpz = rows - rightPoint[1]

            sDistLeft, sDistRight = getDists([lpx, lpz], [rpx, rpz], kinect_depth)

            print ("[atDock.py]: Calling TriangulatePosition.calcTargetPose()"
                   +" for symbol triangulation...\n")
            rSymbol, thetaSymbol = tp.calcTargetPose(IRL_SYM_WIDTH, sDistLeft, sDistRight)
            print ("\n[atDock.py]: ...TriangulatePosition.calcTargetPose()"
                   +" for symbol triangulation finished executing...")

        #Find the position vector to the center of the kinect image frame
            pixelSymbolWidth = rpx - lpx
            camLeftX = int(camX - pixelSymbolWidth/2)
            camRightX = int(camX + pixelSymbolWidth/2)

            #Kinect origin at bottom-left, OpenCV origin at top-left
            kCamZ = rows - camZ
            
            cDistLeft, cDistRight = getDists([camLeftX, kCamZ], [camRightX, kCamZ], kinect_depth)
            
            print ("[atDock.py]: Calling TriangulatePosition.calcTargetPose()"
                   +" for camera center triangulation...")
            rCamera, thetaCamera = tp.calcTargetPose(
                IRL_SYM_WIDTH, cDistLeft, cDistRight)
            print ("[atDock.py]: ...TriangulatePosition.calcTargetPose()"
                   +" for camera center triangulation finished executing...")

            #final calcs
            dTheta = thetaSymbol - thetaCamera
            x = rSymbol*math.cos(dTheta)
            y = rSymbol*math.sin(dTheta)       
            
            dx = int(symX - camX)
            dz = int(symZ - camZ)

            #cv2.imshow('kinect_image', kinect_image)
            #cv2.imshow('outFrame', outFrame)
            
            print('[atDock.py]: Parameters returned: (%f, %f, %f)'
                  % (x, y, thetaCamera))
            return Pose(x, y, thetaCamera)

        except Exception as e:
            print("[atDock.py]: ERROR OCCURED WHILE PROCESSING KINECT IMAGE:")
            print("\t"+str(e))
            
        #finally:
            #print("\n\tPRESS ANY KEY TO CONTINUE...\n")
            #cv2.waitKey(0)
            #v2.destroyAllWindows()

if __name__ == "__main__":
    #=====TESTING=====#  
    print ("[atDock.py]: TEST: Calling \'atDock(kinect_image, kinect_depth)\'... ")
    atDock(cv2.imread('Picture4.jpg', cv2.IMREAD_COLOR), None)
    print ("[atDock.py]: TEST: ...atDock(kinect_image, kinect_depth)\' finished executing. ")
    #=====TESTING=====# 

