#!/usr/bin/env python

import cv2
import numpy as np
import time
import math
import TriangulatePosition as tp
import getLR


IRL_SYM_WIDTH = 0.2032*100 #meters (0.2032m = 8 inches)

class pose():
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

#input Parameters
##image = np.ones((480,640,3), np.uint8)
##image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
##kinect_image = cv2.imread('Marker_Pics/Picture 4.jpg', cv2.IMREAD_COLOR)
##kinect_depth = np.empty((480,640), np.uint8)    #maybe np.uint8
##
##r, c = kinect_depth.shape
##
##cv2.imshow('waht', kinect_depth)
##
##k = kinect_depth.view('float32')
##
##r, c= kinect_depth.shape

def sumPoints(approx):
    x = 0
    y = 0
    
    for i in approx:
        x += i[0][0]
        y += i[0][1]

    return x, y 

def atDock(kinect_image, kinect_depth):
##    cap = cv2.VideoCapture(0)   #0 indicates first webcam in system
##    print('Starting video capture...')
##    time.sleep(3)   #Give camera time to startup
##    print ('Ready for video capture\n')
    # print(type(kinect_image_ros))
    # print(type(kinect_depth_ros))

    print ("atDock(): Starting the atDock() funciton...")
    #TESTING
    kinect_image = cv2.imread('Marker_Pics/Picture 4.jpg', 0)
    print ("atDock(): kinect_image successfully loaded...")
    cv2.waitKey(0)
    cv2.imshow('kinect_image', kinect_image)
    outFrame = kinect_image
    #TESTING   

    
    font = cv2.FONT_HERSHEY_SIMPLEX #font type
    E_COEFF = 0.01   #Coefficient for epsilon value
    SHAPE_CORNERS = 12

    blue = (255,0,0)
    green = (0,255,0)
    red = (0,0,255)
                
    #while (cv2.waitKey(1) & 0xFF) != ord('q'):
    try:
        #_, inFrame = cap.read() #get the video frame

        #outFrame = cv2.cvtColor(kinect_image, cv2.COLOR_RGB2BGR)  #converts to grayscale
        #outFrame = cv2.fastNlMeansDenoising(outFrame,None,50,7,21) #filters out noise (frame, None, higher filter out more noise but gives less detail, filter parameter, filtaer parameter)
        _, mask = cv2.threshold(outFrame, 100, 255, cv2.THRESH_BINARY_INV)    #If pixel value is above 220 it will convert to 255(white), below will turn to black (because binary)
        outFrame = cv2.bitwise_not(mask)
        outFrame = cv2.Canny(outFrame, 50, 50)

        _, contours, _ = cv2.findContours(outFrame,1,2)
        shapes = 0
        symX = 0
        symZ = 0

        print ("atDock(): Looking for Contours in image...")
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt,E_COEFF*cv2.arcLength(cnt,True),True)
            length = len(approx)

            #cv2.drawContours(kinect_image, [cnt], 0, red, -1)
            
            if length == SHAPE_CORNERS:
                shapes += 1
                x, z = sumPoints(approx)
                symX += x
                symZ += z

            #elif not(length == SHAPE_CORNERS):
                #cv2.drawContours(kinect_image, [cnt], 0, red, -1)
        shapes = shapes/2        

        if shapes == 1:

            print ("atDock(): Found a contour! Starting processing...")
            cv2.drawContours(kinect_image, [cnt], 0, red, -1)
            symX /= SHAPE_CORNERS
            symZ /= SHAPE_CORNERS

            rows, cols, _ = kinect_image.shape
            camX = int(cols/2)
            camZ = int(rows/2)

            print ("atDock(): Calling getLR.getLR()...")
            leftPoint, rightPoint = getLR.getLR(approx)
            print ("atDock(): ...getLR.getLR() finished executing")

            #for symbol
            lpx = leftPoint[0]
            lpz = leftPoint[1]
            rpx = rightPoint[0]
            rpz = rightPoint[1]

            sDistLeft = kinect_depth[lpx][lpz]
            sDistRight = kinect_depth[rpx][rpz]

            print ("atDock(): Calling TriangulatePosition.calcTargetPose()"
                   +" for symbol triangulation...")
            rSymbol, thetaSymbol = tp.calcTargetPose(IRL_SYM_WIDTH, sDistLeft, sDistRight)
            print ("atDock(): ...TriangulatePosition.calcTargetPose()"
                   +" for symbol triangulation finished executing...")
            #for Camera
            pixelSymbolWidth = rpx - lpx
            camLeftX = int(camX - pixelSymbolWidth/2)
            camRightX = int(camX + pixelSymbolWidth/2)

            cDistLeft = kinect_depth[camLeftX][camZ]
            cDistRight = kinect_depth[camRightX][camZ]
            
            print ("atDock(): Calling TriangulatePosition.calcTargetPose()"
                   +" for camera center triangulation...")
            rCamera, thetaCamera = tp.calcTargetPose(IRL_SYM_WIDTH, cDistLeft, cDistRight)
            print ("atDock(): ...TriangulatePosition.calcTargetPose()"
                   +" for camera center triangulation finished executing...")

            #final calcs
            dTheta = thetaSymbol - thetaCamera
            x = rSymbol*math.cos(dTheta)
            y = rSymbol*math.sin(dTheta)       
            
            dx = int(symX - camX)
            dz = int(symZ - camZ)

            #cv2.imshow('kinect_image', kinect_image)
            #cv2.imshow('outFrame', outFrame)
            
            print('atDock(): Parameters returned: (%f, %f, %f)' % (x, y, thetaCamera))
            return pose(x, y, thetaCamera)
        
        else:
            cv2.imshow('kinect_image', kinect_image)
            cv2.imshow('outFrame', outFrame)
            cv2.waitKey(0)
            print('atDock(): No Parameters found, \'None\' object returned!')
            return None
                  
    except Exception as e:
        print('ERROR: ' + str(e))

    cv2.waitKey(0)

    #cap.release()   #you gotta release
    cv2.destroyAllWindows()

if __name__ == "__main__":
        print ("Calling \'atDock(kinect_image, kinect_depth)\'... ")
        atDock(kinect_image, kinect_depth)
        print ("...atDock(kinect_image, kinect_depth)\' finished executing. ")


