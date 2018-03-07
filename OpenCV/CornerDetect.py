import cv2
import numpy as np

cap = cv2.VideoCapture(0)   #0 indicates first webcam in system
print('Video Capture initiated')
font = cv2.FONT_HERSHEY_SIMPLEX #font type

while (cv2.waitKey(1) & 0xFF) != ord('q'):
    try:
        ret, frame = cap.read() #ret gets True or false (if an image is being returned), frame gets the image
        #frame  = cv2.fastNlMeansDenoisingColored(frame,None,10,10,7,21)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  #converts to grayscale
        gray  = cv2.fastNlMeansDenoising(gray,None,50,7,21)
        ret, mask = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)    #If pixel value is above 220 it will convert to 255(white), below will turn to black (because binary)
        gray = cv2.bitwise_not(mask)

        #cv2.imshow('gray', gray)

        gray = cv2.Canny(gray, 50, 50)

        corners = cv2.goodFeaturesToTrack(gray, 100, 0.01, 60)  #(image, up to 100 coerners, quality, minimum distance between corners)
        #print(corners)
        if not(corners is None):
            
            corners = np.int0(corners)
            #print(corners)
            numCorners = 0

            for corner in corners:
                x, y = corner.ravel()
                #print('Corner: ' + str(corner) + '\tRavel: ' + str(corner.ravel()))
                cv2.circle(frame, (x,y), 5, (0,0,255) , 1)
                numCorners += 1

            if numCorners <= 24:
                pass
            elif numCorners <= 48:
                cv2.putText(frame, 'Location: 1', (0,440), font, 0.5, (0,255,0), 1, cv2.LINE_AA) #(image, label, start text at, font, size of font, colour(BGR), line width, Anti-aliasing)
            elif numCorners <= 72:
                cv2.putText(frame, 'Location: 2', (0,440), font, 0.5, (0,255,0), 1, cv2.LINE_AA) #(image, label, start text at, font, size of font, colour(BGR), line width, Anti-aliasing)

            C = str(numCorners)
            cv2.putText(frame, 'Number of Corners: ' + C, (0,15), font, 0.5, (0,255,0), 1, cv2.LINE_AA) #(image, label, start text at, font, size of font, colour(BGR), line width, Anti-aliasing)
            
            #cv2.imshow('frame', frame)  #window for original image
            #cv2.imshow('gray and Canny', gray)    #window for grayscale image

        else:
            print('Corners is NONE!')
            #cv2.imshow('frame', frame)  #window for original image
            

    except Exception as e:
        print('ERROR: ' + str(e))
        


cap.release()   #you gotta release
cv2.waitKey(0)
cv2.destroyAllWindows()
