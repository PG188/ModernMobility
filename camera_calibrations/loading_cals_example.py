# Loading the calibration files

import numpy as np

kinect_cals = np.load('kinect_cals.npz')
kinect_intrinsic = kinect_cals['camera']
kinect_distortion = kinect_cals['dist']


webcam_cals = np.load('webcam_cals.npz')
webcam_intrinsic = webcam_cals['camera']
webcam_distortion = webcam_cals['dist']

print(kinect_intrinsic)
print(kinect_distortion)
print(webcam_intrinsic)
print(webcam_distortion)
