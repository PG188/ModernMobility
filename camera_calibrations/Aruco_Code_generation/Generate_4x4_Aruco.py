#! /usr/bin/python
import os
import cv2
from cv2 import aruco
import numpy as np

marker_length = 20 #Any unit. Pose estimation will have the same unit
cal_file = "webcam_calibration.xml"
calibrationParams = cv2.FileStorage(cal_file, cv2.FILE_STORAGE_READ)
camera_matrix = calibrationParams.getNode("cameraMatrix").mat()
dist_coeffs = calibrationParams.getNode("distCoeffs").mat()
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)

for i in range(100):
	ARUCO_SIZE = 800
	ARUCO_ID = i

	img = cv2.aruco.drawMarker(aruco_dict, ARUCO_ID, ARUCO_SIZE, 2)

	cv2.imwrite('Markers_100/Marker_{0}.png'.format(i), img)
