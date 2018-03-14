import numpy as np
import cv2


#ka = np.zeros((480,640,3))
#ka2 = np.ones((480,640), np.float32)

bayer = np.ones((4), np.uint16)
print('bayer\n' + str(bayer))
img = cv2.demosaicing(bayer, cv2.COLOR_BAYER_BG2BGR)
print('\nimg\n' + str(img))


##print(str(ka2))
##cv2.imshow('img', ka2)
##
##ka = cv2.cvtColor(ka2, cv2.COLOR)
#ka = cv2.cvtColor(ka2, cv2.demosaicingCOLOR_BayerBG2BGR)
#print(str(ka))


def GRBG2BRG(ka):
    print(str(ka))
    inFrame = cv2.cvtColor(ka, cv2.COLOR_BayerBG2BGR)
    print(str(inFrame))

#GRBG2BRG(ka)

#img = cv2.imread('RYBG.png', cv2.IMREAD_COLOR)
##print(str(img))
##
##a = np.zeros((3,4))
##print(str(a))

##import numpy as np, cv2
##vis = np.zeros((384, 836), np.float32)
##vis2 = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)
##
##print(str(vis2))
