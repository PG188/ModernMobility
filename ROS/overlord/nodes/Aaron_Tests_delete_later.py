#PLEASE DO NOT CHANGE ANYTHING UNLESS THE COMMENTS STATE OTHERWISE!!!
#====================================================================

import os
import cv2
from cv2 import aruco
import numpy as np
import locWalker
import Mag2D

def AaronTest():
    
    # 1) Corners contains an array of vectors, each of which describes the 
    # locations of the corners of one marker in the frame
    # 2) Strategy is to find the marker closest to the center of the frame, 
    # seperate it from corners, and estimate its pose using 
    # estimatePoseSingleMarkers
    # 3) EstimatePoseSingleMarkers will return rvec, tvec, which describe the 
    #    rotation and translation vectors from the marker to the camera
    #       - The marker coordinate system that is assumed by this function 
    #         is placed at the center of the marker with the Z axis pointing 
    #         out
    # 4) Using the relationship between the pose of the camera and the base_link
    #   frame of the walker, as well as the known pose of that specific marker
    #   (as identified by the id) in the map of the room, we can easily calculate 
    #   the walkers pose in the room map
    # 5) The walkers pose can be returned and then used to determine its navigation
    #    goal

    #Initialization (should not have to do every time)
    marker_length = 0.165 #Any unit. Pose estimation will have the same unit
    #cal_file = "webcam_calibration.xml"
    #calibrationParams = cv2.FileStorage(cal_file, cv2.FILE_STORAGE_READ)
    ##camera_matrix = calibrationParams.getNode("cameraMatrix").mat()
    ##dist_coeffs = calibrationParams.getNode("distCoeffs").mat()
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

    webcam_cals = np.load('webcam_cals.npz')
    camera_matrix = webcam_cals['camera']
    dist_coeffs = webcam_cals['dist']

    ##print(camera_matrix)
    ##print(dist_coeffs)

    NUM_MARKERS = 2 #You are allowed to change this (breaks if you go higher than 20 with size 40 atm)
    ARUCO_SIZE = 40 #You are allowed to change this
    WIN_WIDTH = 480
    WIN_LENGTH = 640
    SPACING = 5

    ids = []

    blank = np.ones((WIN_WIDTH,WIN_LENGTH),np.uint8)*255

    i = 0
    while i < NUM_MARKERS:
        ids.append(cv2.aruco.drawMarker(aruco_dict, i, ARUCO_SIZE, 1))
        xstart = i*(ARUCO_SIZE + SPACING) + SPACING
        ystart = i*(ARUCO_SIZE + SPACING) + SPACING
        
        if xstart > (WIN_WIDTH - ARUCO_SIZE):
            xstart -= (WIN_WIDTH - ARUCO_SIZE)

        if ystart > (WIN_LENGTH - ARUCO_SIZE):
            ystart -= (WIN_LENGTH - ARUCO_SIZE)
        
        xend = xstart + ARUCO_SIZE
        #print(xstart,xend)
        yend = ystart + ARUCO_SIZE
        #print(ystart,yend)
        
        blank[xstart:xend,ystart:yend] = ids[i]
        
        i += 1
        
    #cap = cv2.VideoCapture(0)

    #while (cv2.waitKey(1) & 0xFF != ord('q')):
       # _, frame = cap.read()
        #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    #Read in color image, 'image_color', here
        #image_gray = cv2.cvtColor(image_color, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco.detectMarkers(blank, aruco_dict)

    aruco.drawDetectedMarkers(blank, corners, ids, (120,120,120))

    i = 0
    cm = None #Closest Marker to walker
    mm = None #Marker magnitude
    cmm = None #Comparing marker magnitude
    dx = None #distance vector between walker and marker in on x-axis (Scalar value)
    dy = None #distance vector between walker and marker in on y-axis (Scalar value)
    yaw = None #angle between Aruco's "North" and Walker's "North"

    if not(ids is None): #Markers were detected
        print('Markers detected')
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

        while i < len(ids[0]) + 1:  #This loop determines which marker is the closest
            
            if i == 0:
                mm = Mag2D.Mag2D(tvec[i][0][0], tvec[i][0][1])
                arucoID = ids[i][0]
                dx = tvec[i][0][0]
                dy = tvec[i][0][1]
                yaw = rvec[i][0][0]
                
            else:
                tmp = Mag2D.Mag2D(tvec[i][0][0], tvec[i][0][1])
                
                if tmp > mm:
                    mm = tmp
                    arucoID = ids[i][0]
                    dx = tvec[i][0][0]
                    dy = tvec[i][0][1]
                    yaw = rvec[i][0][0]
                
            print ('rvec[%d] = %s\ntvec[%d] = %s' % (i, rvec[i], i, tvec[i]))
            aruco.drawAxis(blank, camera_matrix, dist_coeffs, rvec[i], tvec[i], marker_length)
            i += 1
        
    else:
        print('No markers detected\n')

    #cv2.imshow('blank', blank)
    cv2.imshow('frame', blank)

    #cap.release()
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    xWalker, yWalker = locWalker.locWalker(arucoID, dx, dy)

    print(xWalker, yWalker, yaw)
    return xWalker, yWalker, yaw

AaronTest()

