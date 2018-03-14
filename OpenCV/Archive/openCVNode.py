#!/usr/bin/env python

import cv2
import numpy as np
import time
import rospy
from std_msgs.msg import Int16

def openCVNode():
	atDock = True
	rospy.init_node('openCVNode', anonymous=True)
	deltax_pub = rospy.Publisher('delta_x', Int16, queue_size = 1)
	deltay_pub = rospy.Publisher('delta_y', Int16, queue_size = 1)
	rate = rospy.Rate(10)	#1Hz
	while not rospy.is_shutdown():
		if atDock:
			deltax, deltay = main()
			deltax_pub.publish(deltax)
			deltay_pub.publish(deltay)
			rate.sleep()

def sumPoints(approx):
    x = 0
    y = 0
    
    for i in approx:
        x += i[0][0]
        y += i[0][1]

    return x, y

def main():

    cap = cv2.VideoCapture(0)   #0 indicates first webcam in system
    print('Starting video capture...')
    time.sleep(3)   #Give camera time to startup
    print ('Ready for video capture\n')

    font = cv2.FONT_HERSHEY_SIMPLEX #font type
    E_COEFF = 0.01   #Coefficient for epsilon value
    SHAPE_CORNERS = 12

    blue = (255,0,0)
    green = (0,255,0)
    red = (0,0,255)
                
    while (cv2.waitKey(1) & 0xFF) != ord('q'):
        try:
            _, inFrame = cap.read() #get the video frame

            print('still working...')
            outFrame = cv2.cvtColor(inFrame, cv2.COLOR_BGR2GRAY)  #converts to grayscale
            #outFrame = cv2.fastNlMeansDenoising(outFrame,None,50,7,21) #filters out noise (frame, None, higher filter out more noise but gives less detail, filter parameter, filtaer parameter)
            _, mask = cv2.threshold(outFrame, 100, 255, cv2.THRESH_BINARY_INV)    #If pixel value is above 220 it will convert to 255(white), below will turn to black (because binary)
            outFrame = cv2.bitwise_not(mask)
            outFrame = cv2.Canny(outFrame, 50, 50)

            _, contours, _ = cv2.findContours(outFrame,1,2)
            shapes = 0
            avgX = 0
            avgY = 0
            for cnt in contours:
                approx = cv2.approxPolyDP(cnt,E_COEFF*cv2.arcLength(cnt,True),True)
                length = len(approx)

                cv2.drawContours(inFrame, [cnt], 0, red, -1)
                
                if length == SHAPE_CORNERS:
                    shapes += 1
                    x, y = sumPoints(approx)
                    avgX += x
                    avgY += y

            if shapes == 1:
				cv2.drawContours(inFrame, [cnt], 0, red, -1)
				avgX /= SHAPE_CORNERS
				avgY /= SHAPE_CORNERS

				rows, cols, _ = inFrame.shape
				camX = cols/2
				camY = rows/2
				dx = int(avgX - camX)
				dy = int(avgY - camY)

				cv2.imshow('inFrame', inFrame)
				cv2.imshow('outFrame', outFrame)

				#print(dx, dy)
				#cv2.imwrite('inFrame.png', inFrame)
				
				print('Delta in pixels: (%d, %d)' % (dx, dy))
				return dx, dy


        except Exception as e:
            print('ERROR: ' + str(e))

    cap.release()   #you gotta release
    cv2.destroyAllWindows()

if __name__ == "__main__":
	try:
		openCVNode()
	except rospy.ROSInterruptException:
		pass
		print('ROS ERROR')
    #main()


