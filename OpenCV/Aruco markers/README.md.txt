# ArUco Markers Localization

Utilizing libraries based around ArUco markers, we can easily detect the presence of these markers in an image
**along** with their orientation. This is all handled by library functions, making the processing itself very 
simple compared to QR codes or other ad-hoc methods. 

The marker_detect code I uploaded is missing necessary features but has a large comment that
describes the overall detection strategy we can use for walker pose detection in a global map. 
Combined with reading some of the pages linked below, the strategy should be reasonably clear. If you have any 
questions, ask me (Josh) 

See the following

General overview: https://docs.opencv.org/3.1.0/d5/dae/tutorial_aruco_detection.html

Library reference: https://longervision.github.io/2017/03/13/opencv-python-aruco/

Detection guide: https://longervision.github.io/2017/03/12/opencv-external-posture-estimation-ArUco-board/
