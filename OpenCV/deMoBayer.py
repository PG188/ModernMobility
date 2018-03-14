import numpy as np
import cv2
import random

bayer = np.ones((480,640), np.uint8)

for i in range(480):
    for j in range(640):
        bayer[i][j] = random.randrange(0,255)

    
print('bayer\n' + str(bayer))
cv2.imshow('bayer', bayer)




def deMoBayer(bayer):
    inFrame = cv2.demosaicing(bayer, cv2.COLOR_BAYER_BG2BGR)

    print('\ninFrame\n' + str(inFrame))
    cv2.imshow('inFrame', inFrame)

deMoBayer(bayer)

