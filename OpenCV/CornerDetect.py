import cv2
import numpy as np

#cap = cv2.VideoCapture(0)   #0 indicates first webcam in system
cap = cv2.VideoCapture(1)   #1 indicates second webcam in system

font = cv2.FONT_HERSHEY_SIMPLEX #font type

blue = (255,0,0)
green = (0,255,0)
red = (0,0,255)

#================================================================================================

def locate(l):
    textLoc = (0,470)

    if (avY >= Bbound):     #cases where target is north of walker
        if (avX <= Lbound):     #target is East of walker
            cv2.putText(inFrame, 'Location: '+ l + ' is NorthEast', textLoc, font, 1, red, 1, cv2.LINE_AA)
        elif (avX >= Rbound):   #target is West of walker
            cv2.putText(inFrame, 'Location: '+ l + ' is NorthWest', textLoc, font, 1, red, 1, cv2.LINE_AA)
        else:                   #target is North of walker
            cv2.putText(inFrame, 'Location: '+ l + ' is North', textLoc, font, 1, red, 1, cv2.LINE_AA)

    elif (avY <= Bbound):   #cases where target is South of walker
        if (avX <= Lbound):     #target is East of walker
            cv2.putText(inFrame, 'Location: '+ l + ' is SouthEast', textLoc, font, 1, red, 1, cv2.LINE_AA)
        elif (avX >= Rbound):   #target is West of walker
            cv2.putText(inFrame, 'Location: '+ l + ' is SouthWest', textLoc, font, 1, red, 1, cv2.LINE_AA)
        else:                   #target is South of walker
            cv2.putText(inFrame, 'Location: '+ l + ' is South', textLoc, font, 1, red, 1, cv2.LINE_AA)

    else:                   #cases where target in-line with the walker
        if (avX <= Lbound):     #target is East of walker
            cv2.putText(inFrame, 'Location: '+ l + ' is East', textLoc, font, 1, red, 1, cv2.LINE_AA)
        elif (avX >= Rbound):   #target is West of walker
            cv2.putText(inFrame, 'Location: '+ l + ' is West', textLoc, font, 1, red, 1, cv2.LINE_AA)
        else:                   #target is centered
            cv2.putText(inFrame, 'Location: '+ l + ' Centered!', textLoc, font, 1, green, 1, cv2.LINE_AA)
        
#================================================================================================
            
while (cv2.waitKey(1) & 0xFF) != ord('q'):
    try:
        ret, inFrame = cap.read() #ret gets True or false (if an image is being returned), frame gets the image
        #cv2.imshow('F', inFrame)

        
        outFrame = cv2.cvtColor(inFrame, cv2.COLOR_BGR2GRAY)  #converts to grayscale
        cv2.imshow('GrayScale', outFrame)
        outFrame  = cv2.fastNlMeansDenoising(outFrame,None,50,7,21) #filters out noise (frame, None, higher filter out more noise but gives less detail, filter parameter, filtaer parameter)
        cv2.imshow('NoiseReduced', outFrame)
        ret, mask = cv2.threshold(outFrame, 100, 255, cv2.THRESH_BINARY_INV)    #If pixel value is above 220 it will convert to 255(white), below will turn to black (because binary)
        outFrame = cv2.bitwise_not(mask)
        cv2.imshow('Masked', outFrame)
        outframe = cv2.Canny(outFrame, 50, 50)

        corners = cv2.goodFeaturesToTrack(outframe, 100, 0.01, 25)  #(image, up to 100 corners, quality, minimum distance between corners)
    
        if not(corners is None):
            
            corners = np.int0(corners)

            numCorners = 0
            avX = 0
            avY = 0

            for corner in corners:
                x, y = corner.ravel()
                cv2.circle(inFrame, (x,y), 5, blue , 1)
                avX = avX + x
                avY = avY + y
                numCorners += 1

            rows, cols, channels = inFrame.shape  #get how many rows and columns in the frams being returned
            Tbound = 0.35*rows
            Bbound = 0.65*rows
            Lbound = 0.35*cols
            Rbound = 0.65*cols

            avX = avX/numCorners
            avY = avY/numCorners

            if numCorners <= 12:
                pass
            
            elif numCorners <= 24:
                locate('1')
                
            elif numCorners <= 36:
                locate('2')

            elif numCorners <= 48:
                locate('3')

            elif numCorners <= 60:
                locate('4')

            C = str(numCorners)
            cv2.putText(inFrame, 'Number of Corners: ' + C, (0,25), font, 1, blue, 1, cv2.LINE_AA) #(image, label, start text at, font, size of font, colour(BGR), line width, Anti-aliasing)
            
            cv2.imshow('inFrame', inFrame)  #window for original image
            cv2.imshow('outFrame', outFrame)

        else:
            print('Corners is NONE!')
            cv2.imshow('inFrame', inFrame)  #window for original image
            cv2.imshow('outFrame', outFrame)

    except Exception as e:
        print('ERROR: ' + str(e))

cap.release()   #you gotta release
cv2.waitKey(0)
cv2.destroyAllWindows()


