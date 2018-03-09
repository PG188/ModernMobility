import cv2
import numpy as np

cap = cv2.VideoCapture(0)   #0 indicates first webcam in system
#cap = cv2.VideoCapture(1)   #1 indicates second webcam in system

font = cv2.FONT_HERSHEY_SIMPLEX #font type
E_COEFF = 0.01   #Coefficient for epsilon value
SHAPE_CORNERS = 12

blue = (255,0,0)
green = (0,255,0)
red = (0,0,255)

#================================================================================================

##def locate(loc):
##    textLoc = (0,470)
##
##    if (avgY >= Bbound):     #Cases where target is north of walker
##        if (avgX <= Lbound):    direction = ' NorthEast'
##        elif (avgX >= Rbound):  direction = ' NorthWest'
##        else:                   direction = ' North'
##
##    elif (avgY <= Bbound):   #Cases where target is South of walker
##        if (avgX <= Lbound):    direction = ' SouthEast'
##        elif (avgX >= Rbound):  direction = ' SouthWest'
##        else:                   direction = ' South'
##
##    else:                   #Cases where target in-line with the walker
##        if (avgX <= Lbound):    direction = ' East'
##        elif (avgX >= Rbound):  direction = ' West'
##        else:                   direction = ' Centered'
##
##    cv2.putText(inFrame, 'Location: ' + str(loc) + ' is ' + direction, textLoc, font, 1, red, 1, cv2.LINE_AA)

def sumPoints(approx):
    x = 0
    y = 0
    
    for i in approx:
        x += i[0][0]
        y += i[0][1]

    return x, y
    
    
#================================================================================================
            
while (cv2.waitKey(1) & 0xFF) != ord('q'):
    try:
        _, inFrame = cap.read() #ret gets True or false (if an image is being returned), frame gets the image
        outFrame = cv2.cvtColor(inFrame, cv2.COLOR_BGR2GRAY)  #converts to grayscale
        outFrame = cv2.fastNlMeansDenoising(outFrame,None,50,7,21) #filters out noise (frame, None, higher filter out more noise but gives less detail, filter parameter, filtaer parameter)
        _, mask = cv2.threshold(outFrame, 100, 255, cv2.THRESH_BINARY_INV)    #If pixel value is above 220 it will convert to 255(white), below will turn to black (because binary)
        outFrame = cv2.bitwise_not(mask)
        outFrame = cv2.Canny(outFrame, 50, 50)

        _,contours,_ = cv2.findContours(outFrame,1,2)
        shapes = 0
        avgX = 0
        avgY = 0
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt,E_COEFF*cv2.arcLength(cnt,True),True)
            length = len(approx)
            
            if length == SHAPE_CORNERS:
                cv2.drawContours(inFrame, [cnt], 0, red, -1)
                shapes += 1
                x, y = sumPoints(approx)
                avgX += x
                avgY += y

        if shapes == 2:
            avgX /= 2*SHAPE_CORNERS
            avgY /= 2*SHAPE_CORNERS
            print avgX, avgY
            
            
        cv2.imshow('inFrame', inFrame)
        cv2.imshow('outFrame', outFrame)

    except Exception as e:
        print('ERROR: ' + str(e))

cap.release()   #you gotta release
cv2.destroyAllWindows()


