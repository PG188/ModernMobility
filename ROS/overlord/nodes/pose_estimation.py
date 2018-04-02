#PLEASE DO NOT CHANGE ANYTHING UNLESS THE COMMENTS STATE OTHERWISE!!!
#====================================================================

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

import os
import cv2
from cv2 import aruco
import numpy as np
import locWalker
import Mag3D
import ReadMap

def marker_detect():
    
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

def get_pose(location):
    if location == 'walker':
        return marker_detect()
    else:
        return ReadMap.getConstPose(location)
    

