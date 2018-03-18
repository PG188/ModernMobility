#Take in list of points, returns left and right most points from list

import numpy as np

def getLR(approx):
    L = 0
    R = 0
    lp = [0,0]
    rp = [0,0]

    for i in range(len(approx)):

        point = [approx[i][0][0], approx[i][0][1]]
        
        if point[0] < L:
            L = point[0]
            lp = point
        
        if point[0] > R:
            R = point[0]
            rp = point

    return lp, rp
