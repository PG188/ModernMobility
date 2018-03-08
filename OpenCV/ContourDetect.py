import cv2
import numpy as np

cap = cv2.VideoCapture(0)   #0 indicates first webcam in system
#cap = cv2.VideoCapture(1)   #1 indicates second webcam in system

font = cv2.FONT_HERSHEY_SIMPLEX #font type
E_COEFF = 0.01   #Coefficient for epsilon value

blue = (255,0,0)
green = (0,255,0)
red = (0,0,255)

#================================================================================================

def locate(loc):
    textLoc = (0,470)

    if (avgY >= Bbound):     #Cases where target is north of walker
        if (avgX <= Lbound):    direction = ' NorthEast'
        elif (avgX >= Rbound):  direction = ' NorthWest'
        else:                   direction = ' North'

    elif (avgY <= Bbound):   #Cases where target is South of walker
        if (avgX <= Lbound):    direction = ' SouthEast'
        elif (avgX >= Rbound):  direction = ' SouthWest'
        else:                   direction = ' South'

    else:                   #Cases where target in-line with the walker
        if (avgX <= Lbound):    direction = ' East'
        elif (avgX >= Rbound):  direction = ' West'
        else:                   direction = ' Centered'

    cv2.putText(inFrame, 'Location: ' + str(loc) + ' is ' + direction, textLoc, font, 1, red, 1, cv2.LINE_AA)
        
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
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt,E_COEFF*cv2.arcLength(cnt,True),True)
            length = len(approx)
            #print "\nPoints found = ", length
            if length == 12:
                cv2.drawContours(inFrame, [cnt], 0, (0,0,255), -1)

        cv2.imshow('inFrame', inFrame)
        cv2.imshow('outFrame', outFrame)


            
##        corners = cv2.goodFeaturesToTrack(outFrame, 100, 0.01, 60)  #(image, up to 100 corners, quality, minimum distance between corners)
##
##        if not(corners is None):
##            
##            corners = np.int0(corners)
##
##            numCorners = 0
##            avgX = 0
##            avgY = 0
##
##            for corner in corners:
##                x, y = corner.ravel()
##                cv2.circle(inFrame, (x,y), 5, blue , 1)
##                avgX += x
##                avgY += y
##                numCorners += 1
##
##            rows, cols, channels = inFrame.shape  #get how many rows and columns in the frams being returned
##            Tbound = 0.35*rows
##            Bbound = 0.65*rows
##            Lbound = 0.35*cols
##            Rbound = 0.65*cols
##
##            avgX /= numCorners
##            avgY /= numCorners
##
##            if numCorners <= 12:
##                pass
##            
##            elif numCorners <= 24:
##                locate('1')
##                
##            elif numCorners <= 36:
##                locate('2')
##
##            elif numCorners <= 48:
##                locate('3')
##
##            elif numCorners <= 60:
##                locate('4')
##
##            C = str(numCorners)
##            cv2.putText(inFrame, 'Number of Corners: ' + C, (0,25), font, 1, blue, 1, cv2.LINE_AA) #(image, label, start text at, font, size of font, colour(BGR), line width, Anti-aliasing)
##            
##            cv2.imshow('inFrame', inFrame)  #window for original image
##            cv2.imshow('outFrame', outFrame)
##
##        else:
##            print('Corners is NONE!')
##            cv2.imshow('inFrame', inFrame)  #window for original image
##            cv2.imshow('outFrame', outFrame)

    except Exception as e:
        print('ERROR: ' + str(e))

cap.release()   #you gotta release
cv2.waitKey(0)
cv2.destroyAllWindows()


