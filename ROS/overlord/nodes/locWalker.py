
def locWalker(arucoID, dx, dy, dtheta):
    xid, yid = getXY(arucoID)
    xWalker = xid - dx
    yWalker = yid - dy

    return xWalker, yWalker, dtheta
    
