import ReadMap

def locWalker(arucoID, dx, dy, dtheta):
    xid, yid = ReadMap.getXY(arucoID)
    xWalker = xid - dx
    yWalker = yid - dy

    return xWalker, yWalker, dtheta

    
