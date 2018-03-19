#data_out = list(pc2.read_points(kinect_depth, field_names=("x", "y", "z"), skip_nans=True, uvs=[[1, 1], [600,1], [600, 400], [1, 400]]))

#!/usr/bin/env python

import cv2
import numpy as np
import time
import math
import TriangulatePosition as tp
import getLR
import sensor_msgs.point_cloud2 as pc2


IRL_SYM_WIDTH = 0.2032 #meters (0.2032m = 8 inches) 

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
##   cap = cv2.VideoCapture(0)   #0 indicates first webcam in system
##    print('Starting video capture...')
##    time.sleep(3)   #Give camera time to startup
##    print ('Ready for video capture\n')
    #print(type(kinect_image))
    # print(type(kinect_depth_ros)) 
    #outFrame = np.array(kinect_image)


    print ("atDock(): Starting the atDock() funciton...")
    #TESTING
    #kinect_image = cv2.imread('/home/josh/catkin_ws/Picture4.jpg', cv2.IMREAD_COLOR)

    #cv2.imshow('kinect', kinect_image)
    #cv2.waitKey(0)
    #outFrame = kinect_image
    #TESTING   
    
    font = cv2.FONT_HERSHEY_SIMPLEX #font type
    E_COEFF = 0.01   #Coefficient for epsilon value
    SHAPE_CORNERS = 12

    blue = (255,0,0)
    green = (0,255,0)
    red = (0,0,255)
                
    #while (cv2.waitKey(1) & 0xFF) != ord('q'):
    print(0)
    try:
        #_, inFrame = cap.read() #get the video frame
        outFrame = cv2.cvtColor(kinect_image, cv2.COLOR_BGR2GRAY)  #converts to grayscale
        #outFrame = cv2.fastNlMeansDenoising(outFrame,None,50,7,21) #filters out noise (frame, None, higher filter out more noise but gives less detail, filter parameter, filtaer parameter)
        print(1)

        _, mask = cv2.threshold(outFrame, 100, 255, cv2.THRESH_BINARY_INV)    #If pixel value is above 220 it will convert to 255(white), below will turn to black (because binary)
        outFrame = cv2.bitwise_not(mask)
        outFrame = cv2.Canny(outFrame, 50, 50)

        cv2.imshow('checkCheck', outFrame)
        cv2.waitKey(0)

        print(2)

        _, contours, _ = cv2.findContours(outFrame,1,2)
        shapes = 0
        symX = 0
        symZ = 0

        #cv2.imshow('outFrame', outFrame)

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
        print(3)

        shapes = shapes/2

        print("[atDock]: Shapes: {}".format(shapes))

        if not(shapes == 1):
            shapes = 1        

        

        if shapes == 1:

            print(4)

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

            print(leftPoint, rightPoint)

            print(5)

            #for symbol
            lpx = leftPoint[0]
            lpz = rows - leftPoint[1]   #Kinect origin at bottom-left, OpenCV origin at top-left
            rpx = rightPoint[0]
            rpz = rows - rightPoint[1]  #Kinect origin at bottom-left, OpenCV origin at top-left

##            sDistLeft = kinect_depth[lpx][lpz]
##            sDistRight = kinect_depth[rpx][rpz]
    
            print("lpx:{}  lpz:{} rpx:{}  rpz:{}".format(lpx, lpz, rpx, rpz))    

            print(6)


            data_out4symbol = list(pc2.read_points(kinect_depth, field_names=("x", "y", "z"), skip_nans=False, uvs=[[lpx, lpz], [rpx, rpz]]))    #[[(x,y,z)]]
            print('(x, y, z) from depth: %s' %(data_out4symbol))
            sDistLeft = data_out4symbol[0][2]
            sDistRight = data_out4symbol[1][2]

            print(7)

            print ("atDock(): Calling TriangulatePosition.calcTargetPose()"
                   +" for symbol triangulation...")
            rSymbol, thetaSymbol = tp.calcTargetPose(IRL_SYM_WIDTH, sDistLeft, sDistRight)
            print ("atDock(): ...TriangulatePosition.calcTargetPose()"
                   +" for symbol triangulation finished executing...")

            print(8)

            #for Camera
            pixelSymbolWidth = rpx - lpx
            camLeftX = int(camX - pixelSymbolWidth/2)
            camRightX = int(camX + pixelSymbolWidth/2)
            kCamZ = rows - camZ     #Kinect origin at bottom-left, OpenCV origin at top-left

##            cDistLeft = kinect_depth[camLeftX][camZ]
##            cDistRight = kinect_depth[camRightX][camZ]

            data_out4camera = list(pc2.read_points(kinect_depth, field_names=("x", "y", "z"), skip_nans=True, uvs=[[camLeftX, kCamZ], [camRightX, kCamZ]]))    #[[(x,y,z)]]
            cDistLeft = data_out4camera[0][2]
            cDistRight = data_out4camera[1][2]

            print(9)
            
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

            print('atDock(): Parameters returned: (x, y, theta) = (%f, %f, %f)' % (x, y, thetaCamera))

            cv2.imshow('kinect_image', kinect_image)
            cv2.imshow('outFrame', outFrame)
            cv2.waitKey(0)
            #cap.release()   #you gotta release
            cv2.destroyAllWindows()
            
            print('atDock(): Parameters returned: (x, y, theta) = (%f, %f, %f)' % (x, y, thetaCamera))
            return pose(x, y, thetaCamera)
        
        else:
            cv2.imshow('kinect_image', kinect_image)
            cv2.imshow('outFrame', outFrame)
            cv2.waitKey(0)
            print('atDock(): No Parameters found, \'None\' object returned!')
            #cap.release()   #you gotta release
            cv2.destroyAllWindows()
            return None
                  
    except Exception as e:
        print('ERROR (atDock except): ' + str(e))

    

if __name__ == "__main__":
        print ("Calling \'atDock(kinect_image, kinect_depth)\'... ")
        atDock(kinect_image, kinect_depth)
        print ("...atDock(kinect_image, kinect_depth)\' finished executing. ")


