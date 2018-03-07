import cv2
import numpy as np

cap = cv2.VideoCapture(0)   #0 indicates first webcam in system
print('Video Capture initiated')
font = cv2.FONT_HERSHEY_SIMPLEX #font type

while (cv2.waitKey(1) & 0xFF) != ord('q'):
    try:
        ret, inFrame = cap.read() #ret gets True or false (if an image is being returned), frame gets the image
        outFrame = cv2.cvtColor(inFrame, cv2.COLOR_BGR2GRAY)  #converts to grayscale
        outFrame  = cv2.fastNlMeansDenoising(outFrame,None,50,7,21) #filters out noise (frame, None, higher filter out more noise but gives less detail, filter parameter, filtaer parameter)
        ret, mask = cv2.threshold(outFrame, 100, 255, cv2.THRESH_BINARY_INV)    #If pixel value is above 220 it will convert to 255(white), below will turn to black (because binary)
        outFrame = cv2.bitwise_not(mask)
        outframe = cv2.Canny(outFrame, 50, 50)

        corners = cv2.goodFeaturesToTrack(outframe, 100, 0.01, 60)  #(image, up to 100 coerners, quality, minimum distance between corners)
        #print(corners)
        if not(corners is None):
            
            corners = np.int0(corners)

            numCorners = 0
            avX = 0
            avY = 0

            for corner in corners:
                x, y = corner.ravel()
                cv2.circle(inFrame, (x,y), 5, (255,0,0) , 1)
                avX = avX + x
                avY = avY + y
                numCorners += 1

            rows, cols, channels = inFrame.shape  #get how many rows and columns in the frams being returned
            Tbound = 0.4*rows
            Bbound = 0.6*rows
            Lbound = 0.4*cols
            Rbound = 0.6*cols

            avX = avX/numCorners
            avY = avY/numCorners

            if numCorners <= 2:
                pass
            
            elif numCorners <= 6:

                if (avX > Lbound) and (avX < Rbound) and (avY > Tbound) and (avY < Bbound):
                    cv2.putText(inFrame, 'Location: 1 Centred', (0,440), font, 1, (0,255,0), 1, cv2.LINE_AA) #(image, label, start text at, font, size of font, colour(BGR), line width, Anti-aliasing)
                else:
                    if (avY >= Bbound):
                        cv2.putText(inFrame, 'Location: 1 is North', (0,440), font, 1, (0,0,255), 1, cv2.LINE_AA) #(image, label, start text at, font, size of font, colour(BGR), line width, Anti-aliasing)
                    
                    elif (avX <= Lbound):
                        cv2.putText(inFrame, 'Location: 1 is East', (0,440), font, 1, (0,0,255), 1, cv2.LINE_AA) #(image, label, start text at, font, size of font, colour(BGR), line width, Anti-aliasing)
                        
                    elif (avX >=Rbound):
                        cv2.putText(inFrame, 'Location: 1 is West', (0,440), font, 1, (0,0,255), 1, cv2.LINE_AA) #(image, label, start text at, font, size of font, colour(BGR), line width, Anti-aliasing)

                    else:
                        cv2.putText(inFrame, 'Location: 1 is South', (0,440), font, 1, (0,0,255), 1, cv2.LINE_AA) #(image, label, start text at, font, size of font, colour(BGR), line width, Anti-aliasing)
                
            elif numCorners <= 10:
                if (avX > Lbound) and (avX < Rbound) and (avY > Tbound) and (avY < Bbound):
                    cv2.putText(inFrame, 'Location: 1 Centred', (0,440), font, 1, (0,255,0), 1, cv2.LINE_AA) #(image, label, start text at, font, size of font, colour(BGR), line width, Anti-aliasing)
                else:
                    if (avY >= Bbound):
                        cv2.putText(inFrame, 'Location: 1 is North', (0,440), font, 1, (0,0,255), 1, cv2.LINE_AA) #(image, label, start text at, font, size of font, colour(BGR), line width, Anti-aliasing)
                    
                    elif (avX <= Lbound):
                        cv2.putText(inFrame, 'Location: 1 is East', (0,440), font, 1, (0,0,255), 1, cv2.LINE_AA) #(image, label, start text at, font, size of font, colour(BGR), line width, Anti-aliasing)
                        
                    elif (avX >=Rbound):
                        cv2.putText(inFrame, 'Location: 1 is West', (0,440), font, 1, (0,0,255), 1, cv2.LINE_AA) #(image, label, start text at, font, size of font, colour(BGR), line width, Anti-aliasing)

                    else:
                        cv2.putText(inFrame, 'Location: 1 is South', (0,440), font, 1, (0,0,255), 1, cv2.LINE_AA) #(image, label, start text at, font, size of font, colour(BGR), line width, Anti-aliasing)

            C = str(numCorners)
            cv2.putText(inFrame, 'Number of Corners: ' + C, (0,15), font, 0.5, (255,0,0), 1, cv2.LINE_AA) #(image, label, start text at, font, size of font, colour(BGR), line width, Anti-aliasing)
            
            cv2.imshow('inFrame', inFrame)  #window for original image
            cv2.imshow('outFrame', outFrame)    #window for grayscale image

        else:
            print('Corners is NONE!')
            #cv2.imshow('frame', frame)  #window for original image
            

    except Exception as e:
        print('ERROR: ' + str(e))
        


cap.release()   #you gotta release
cv2.waitKey(0)
cv2.destroyAllWindows()
