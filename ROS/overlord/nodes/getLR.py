#Take in list of points, returns left and right most points from list

import numpy as np

def getLR(approx):
    try:
        L = approx[0][0][0]
        R = approx[0][0][0]
        lp = approx[0][0]
        rp = approx[0][0]
        point = None

        print ("getLR(): Showing iterative data:")
        for i in range(1,len(approx)):
            point = [approx[i][0][0], approx[i][0][1]]
            
            if point[0] < L:
                L = point[0]
                lp = point
            
            if point[0] > R:
                R = point[0]
                rp = point

            print ("\tpoint = %s | L = %s | R = %s | lp = %s | rp = %s"
                   % (point, L, R, lp, rp))

        print ("getLR(): Successfully returned points")
        return lp, rp
    except:
        print ("getLR(): No points found!")
        return None, None
