import math
"""
Reference for going from opencv to quaternion:
https://stackoverflow.com/questions/12933284/rodrigues-into-eulerangles-and-vice-versa
"""

##class Quaternion:
##    def __init__(self, w, x, y, z):
##        self.w = w
##        self.x = x
##        self.y = y
##        self.z = z
##
###==========Private=Functions==========#
##def _isNorthPoleSingularity(q):
##    #when qx*qy + qz*qw = 0.5 (north pole)
##    return (q.x*q.y + q.z*q.w > 4.999)
##
##def _isSouthPoleSingularity(q):
##    #when qx*qy + qz*qw = -0.5 (south pole)
##    return (q.x*q.y + q.z*q.w < -4.999)
##
##def _quaternion2Yaw(q):
##    #if north: 2 * atan2(x,w)
##    if _isNorthPoleSingularity(q): return 2*math.atan2(q.x,q.w)
##
##    #if south: -2 * atan2(x,w)
##    elif _isSouthPoleSingularity(q): return -2*math.atan2(q.x,q.w)
##
##    #otherwise: atan2(2*qy*qw-2*qx*qz , 1 - 2*qy^2 - 2*qz^2)
##    else:
##        numer = 2 * (q.y*q.w - q.x*q.z)
##        denom = 1 - 2 * (q.y*q.y - q.z*q.z)
##        return math.atan2(numer, denom)
##
##def _quaternion2Pitch(q):
##    #always: asin(2*qx*qy + 2*qz*qw) 
##    return math.asin(2*q.x*q.y + 2*q.z*q.w) 
##
##def _quaternion2Roll(q):
##    #if north or South: 0
##    if _isNorthPoleSingularity(q) or _isSouthPoleSingularity(q): return 0
##
##    #otherwise: atan2(2*qx*qw-2*qy*qz , 1 - 2*qx^2 - 2*qz^2)
##    else:
##        numer = 2 * (q.x*q.w - q.y*q.z)
##        denom = 1 - 2 * (q.x*q.x - q.z*q.z)
##        return math.atan2(numer, denom)
##
##def _rvec2quaternion(a, b, c):
##    w = mag3D(a,b,c)
##    x = a/w
##    y = b/w
##    z = c/w
##    print('[custom_math.py]: _rvec2quaternion():\nQ:[%s, %s, %s, %s]' % (w,x,y,z))
##    return Quaternion(w,x,y,z)
##
##def _rvec2YPR(i, rvec):
##    q = _rvec2quaternion(rvec[i][0][0], rvec[i][0][1], rvec[i][0][2])
##    return _quaternion2Yaw(q), _quaternion2Pitch(q), _quaternion2Roll(q)

#==========Public=Functions==========#

##def getYawFromRvec(i, rvec):
##    return _rvec2YPR(i, rvec)[0]

def mag3D(x, y, z):
    return math.sqrt(x*x + y*y + z*z)

def deg2rad(deg_val):
    return math.pi*deg_val/180.0

def rad2deg(rad_val):
    return 180.0*rad_val/math.pi

def reverseRotDir(theta):
    if(theta < 0): return -1*theta - math.pi
    else: return -1*theta + math.pi
