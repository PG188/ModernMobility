import os
import cv2
from cv2 import aruco
import numpy as np

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
marker_length = 20 #Any unit. Pose estimation will have the same unit
cal_file = "webcam_calibration.xml"
calibrationParams = cv2.FileStorage(cal_file, cv2.FILE_STORAGE_READ)
camera_matrix = calibrationParams.getNode("cameraMatrix").mat()
dist_coeffs = calibrationParams.getNode("distCoeffs").mat()
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

#Read in color image, 'image_color', here
image_gray = cv2.cvtColor(image_color, cv2.COLOR_BGR2GRAY)
corners, ids = aruco.detectMarkers(image_gray, aruco_dict)
if ids != None: #Markers were detected
    print('Markers detected\n')
    rvec, tvec = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

else:
    print('No markers detected\n')

