#pose_estimation.py version 2

"""
PLEASE DO NOT CHANGE ANYTHING UNLESS THE COMMENTS STATE OTHERWISE!!!
====================================================================

 1) Corners contains an array of vectors, each of which describes the 
 locations of the corners of one marker in the frame
 2) Strategy is to find the marker closest to the center of the frame, 
 seperate it from corners, and estimate its pose using 
 estimatePoseSingleMarkers
 3) EstimatePoseSingleMarkers will return rvec, tvec, which describe the 
    rotation and translation vectors from the marker to the camera
       - The marker coordinate system that is assumed by this function 
         is placed at the center of the marker with the Z axis pointing 
         out
 4) Using the relationship between the pose of the camera and the base_link
   frame of the walker, as well as the known pose of that specific marker
   (as identified by the id) in the map of the room, we can easily calculate 
   the walkers pose in the room map
 5) The walkers pose can be returned and then used to determine its navigation
    goal
"""

import os
import cv2
from cv2 import aruco
import numpy as np
import custom_math
import ReadMap
from TransformMatrix import *
from getpass import getuser
import sys

#User configurable inputs
TEST_MODE = True
VIDEO_CAP_CHANNEL = 1  #For raspberry pi use 0, laptop use 1
SHOW_CAPTURED_FRAME = TEST_MODE
FRAME_CAP_ATTEMPTS = 5

#Constants
NAN = float('NaN')
MARKER_LENGTH = 0.165   #meters

#Camera to Walker frame differences
W2C_X = -0.486664    #In meters
W2C_Y = 0           #In meters
W2C_Z = -0.854964    #In meters
W2C_YAW = 0         #In radians

#variable settings    
COMPUTER = getuser()
if COMPUTER == 'fauziakhanum':
    WEBCAM_CALS_PATH = '/home/fauziakhanum/catkin_ws/src/src/overlord/nodes/webcam_cals.npz'
elif COMPUTER == 'josh':
    WEBCAM_CALS_PATH = '/home/josh/catkin_ws/src/overlord/nodes/webcam_cals.npz'
else:
    WEBCAM_CALS_PATH = 'webcam_cals.npz'
#====================Private Functions====================#
def _locWalker(arucoID, dx, dy):
    xid, yid, _ = ReadMap.getPose(arucoID)
    xWalker = xid - dx
    yWalker = yid - dy

    return xWalker, yWalker

def _camBase2walkerBase(cam_x, cam_y, cam_yaw):
    #Create transformation matrix to go from camera frame to walker frame
    tm = TransformMatrix()
    tm.translate(W2C_X, W2C_Y, W2C_Z)   #Deal with any x or y translations
    tm.rotate(Axis.Z, W2C_YAW)          #Deal with any yaw rotations
    
    #Translate vector to walker frame
    walker_x, walker_y, _ = tm.transformVector(cam_x, cam_y)
    return walker_x, walker_y, (custom_math.reverseRotDir(cam_yaw) + W2C_YAW)

def _rvec2YPR(i, rvec):
    #converts rvec into rodrigues rotation matrix
    rod_mat = cv2.Rodrigues(rvec[i][0])[0]

    #construct a 3X4 projection matrix using the rodrigues matrix
    proj_mat = [[rod_mat[0][0], rod_mat[0][1], rod_mat[0][2], 0],
                [rod_mat[1][0], rod_mat[1][1], rod_mat[1][2], 0],
                [rod_mat[2][0], rod_mat[2][1], rod_mat[2][2], 0]]

    #get array of euler angles in degrees
    euler_angles = cv2.decomposeProjectionMatrix(np.array(proj_mat))[6]

    #return in reverse order to get Yaw, Pitch, Roll
    return euler_angles[2][0], euler_angles[1][0], euler_angles[0][0]

def _getYaw(i, rvec):
    #return yaw value in rads
    return custom_math.deg2rad(_rvec2YPR(i, rvec)[0])

def _marker_detect(failed_detections = 0):
    
    marker_length = MARKER_LENGTH   #Any unit. Pose estimation will have the same unit
    
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)

    webcam_cals = np.load(WEBCAM_CALS_PATH)
    camera_matrix = webcam_cals['camera']
    dist_coeffs = webcam_cals['dist']

    xWalker = NAN   #Walker's x position in the map
    yWalker = NAN   #Walker's y position in the map
    i = 0
    mm = -1     #Magnitue of vector between Walker camera and aruco marker (in meters)
    dx = NAN    #distance vector between walker and marker in on x-axis (Scalar value)
    dy = NAN    #distance vector between walker and marker in on y-axis (Scalar value)
    yaw = NAN   #angle between Aruco's "North" and Walker's "North"
    arucoID = -1
        
    cap = cv2.VideoCapture(VIDEO_CAP_CHANNEL)
    _, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict)

    aruco.drawDetectedMarkers(frame, corners, ids, (120,120,120))

    if not(ids is None): #Markers were detected
        print('\n[pose_estimation.py]:marker_detect():\tMarkers detected')

        py_ver = sys.version_info[0]
        if (py_ver == 2): rvec, tvec = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
        elif (py_ver == 3): rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
        else: print("\n[pose_estimation.py]:_marker_detect():\nMust be using python version 2 or 3\n")

        while i < len(ids):  #This loop determines which marker is the closest
            
            if i == 0:
                mm = custom_math.mag3D(tvec[i][0][0], tvec[i][0][1], tvec[i][0][2])
                tmp = mm
                arucoID = ids[i][0]
                dx = tvec[i][0][0]
                dy = tvec[i][0][1]
                yaw = _getYaw(i, rvec)
                
            else:
                tmp = custom_math.mag3D(tvec[i][0][0], tvec[i][0][1],tvec[i][0][2])
                
                if tmp < mm:
                    mm = tmp
                    arucoID = ids[i][0]
                    dx = tvec[i][0][0]
                    dy = tvec[i][0][1]
                    yaw = _getYaw(i, rvec)
                
            print ('\n[pose_estimation.py]:marker_detect():\n    i = %d\n    Magnitude = %f\n    arucoID = %d\n    rvec[%d] = %s\n    tvec[%d] = %s' %(i, tmp, ids[i][0], i, rvec[i], i, tvec[i]))
            aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec[i], tvec[i], marker_length)
            i += 1

        #Show frame, not needed for actual processing
        if(SHOW_CAPTURED_FRAME):
            cv2.imshow('frame', frame)
            cap.release()
            cv2.waitKey(0)

        #After detection is done get the Walker's x and y position
        xWalker, yWalker = _locWalker(arucoID, dx, dy)
    
    else:
        print('\n[pose_estimation.py]:_marker_detect():\tNo markers detected\n')
        failed_detections += 1
        print('[pose_estimation.py]:_marker_detect():\tFailed to detect marker %s time(s)' % failed_detections)
        
        if(failed_detections < FRAME_CAP_ATTEMPTS):
            print('[pose_estimation.py]:_marker_detect():\tReleasing captured frame...')
            cap.release()
            cv2.destroyAllWindows()

            print('[pose_estimation.py]:_marker_detect():\tRetrying _marker_detect()...')
            return _marker_detect(failed_detections)
            
        else:
            print('[pose_estimation.py]:_marker_detect():\tReached max number of retries (%s)' % failed_detections)
            print('[pose_estimation.py]:_marker_detect():\tCould not find or detect any markers!')
            print('\n[pose_estimation.py]:_marker_detect():\n Final Values:\n     Magnitude = %f\n     arucoID = %d\n     dx = %f\n     dy = %f\n     yaw = %f' %(mm, arucoID, dx, dy, yaw))
            return NAN, NAN, NAN

    cap.release()
    cv2.destroyAllWindows()
    
    return xWalker, yWalker, yaw

#====================Public Functions====================#
"""
get_pose():
    Inputs:
        location <str>: name of the location being requested

    Returns:
        (walker_xpos, walker_ypos, walker_yaw) <float, float, float>: Python tuple
        containing the Walker's x position, y position, and yaw orientation 
        respectively
"""
def get_pose(location):

    #If looking for walker location we must use the ArUco markers
    if location == 'walker':
        cam_x, cam_y, cam_yaw = _marker_detect()            #Find pose from marker
        return _camBase2walkerBase(cam_x, cam_y, cam_yaw)   #Translate to walker frame
    
    #If looking for a different location, it is expected to be a static location
    #Specified in the map_constants.json file
    else:
        return ReadMap.getConstPose(location)

##TEST
if TEST_MODE:
    print('\n')
    print(get_pose('walker'))
