#PLEASE DO NOT CHANGE ANYTHING UNLESS THE COMMENTS STATE OTHERWISE!!!
#====================================================================

import os
import cv2
from cv2 import aruco
import numpy as np
import locWalker
import Mag3D

def TestImage():
    
    #Initialization (should not have to do every time)
    marker_length = 0.165 #Any unit. Pose estimation will have the same unit
    
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

    webcam_cals = np.load('webcam_cals.npz')
    camera_matrix = webcam_cals['camera']
    dist_coeffs = webcam_cals['dist']

    NUM_MARKERS = 5 #You are allowed to change this (breaks if you go higher than 20 with size 40 atm)
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
        
        yend = ystart + ARUCO_SIZE
        
        blank[xstart:xend,ystart:yend] = ids[i]
        
        i += 1
        
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
        print('\nTestImage:\tMarkers detected')
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

        while i < len(ids):  #This loop determines which marker is the closest
            
            if i == 0:

                mm = Mag3D.Mag3D(tvec[i][0][0], tvec[i][0][1],tvec[i][0][2])
                tmp = mm
                arucoID = ids[i][0]
                dx = tvec[i][0][0]
                dy = tvec[i][0][1]
                yaw = rvec[i][0][0]
                
            else:
                tmp = Mag3D.Mag3D(tvec[i][0][0], tvec[i][0][1],tvec[i][0][2])

                if tmp < mm:
                    mm = tmp
                    arucoID = ids[i][0]
                    dx = tvec[i][0][0]
                    dy = tvec[i][0][1]
                    yaw = rvec[i][0][0]
                
            print ('\nTestImage:\ti = %d:\t Magnitude = %f, arucoID = %d, rvec[%d] = %s\ttvec[%d] = %s' %(i, tmp, ids[i][0], i, rvec[i], i, tvec[i]))
            aruco.drawAxis(blank, camera_matrix, dist_coeffs, rvec[i], tvec[i], marker_length)
            i += 1
        
    else:
        print('\nTestImage:\tNo markers detected\n')

    print('\nTestImage:\tFinal Values:\tMagnitude = %f, arucoID = %d, dx = %f, dy = %f, yaw = %f' %(mm, arucoID, dx, dy, yaw))

    cv2.imshow('blank', blank)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

    xWalker, yWalker = locWalker.locWalker(arucoID, dx, dy)

    
    return xWalker, yWalker, yaw

def TestVideo():
    
    marker_length = 0.165 #Any unit. Pose estimation will have the same unit
    
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

    webcam_cals = np.load('webcam_cals.npz')
    camera_matrix = webcam_cals['camera']
    dist_coeffs = webcam_cals['dist']

    i = 0
    cm = None #Closest Marker to walker
    mm = None #Marker magnitude
    cmm = None #Comparing marker magnitude
    dx = None #distance vector between walker and marker in on x-axis (Scalar value)
    dy = None #distance vector between walker and marker in on y-axis (Scalar value)
    yaw = None #angle between Aruco's "North" and Walker's "North"
        
    cap = cv2.VideoCapture(1)

    #while (cv2.waitKey(1) & 0xFF != ord('q')):
    _, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)   
    
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict)

    aruco.drawDetectedMarkers(frame, corners, ids, (120,120,120))

    if not(ids is None): #Markers were detected
        print('\nTestVideo:\tMarkers detected')
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

        while i < len(ids):  #This loop determines which marker is the closest
            
            if i == 0:
                mm = Mag3D.Mag3D(tvec[i][0][0], tvec[i][0][1], tvec[i][0][2])
                tmp = mm
                arucoID = ids[i][0]
                dx = tvec[i][0][0]
                dy = tvec[i][0][1]
                yaw = rvec[i][0][0]
                
            else:
                tmp = Mag3D.Mag3D(tvec[i][0][0], tvec[i][0][1],tvec[i][0][2])
                
                if tmp < mm:
                    mm = tmp
                    arucoID = ids[i][0]
                    dx = tvec[i][0][0]
                    dy = tvec[i][0][1]
                    yaw = rvec[i][0][0]
                
            print ('\nTestVideo:\ti = %d, Magnitude = %f, arucoID = %d, rvec[%d] = %s\ttvec[%d] = %s' %(i, tmp, ids[i][0], i, rvec[i], i, tvec[i]))
            aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec[i], tvec[i], marker_length)
            i += 1

    else:
        print('\nTestVideo:\tNo markers detected\n')

    print('\nTestIVideo:\tFinal Values:\tMagnitude = %f, arucoID = %d, dx = %f, dy = %f, yaw = %f' %(mm, arucoID, dx, dy, yaw))

    cv2.imshow('frame', frame)

    cap.release()
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    xWalker, yWalker = locWalker.locWalker(arucoID, dx, dy)    
    return xWalker, yWalker, yaw

#TestImage()
TestVideo()
    

