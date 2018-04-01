import ReadMap

def locWalker(arucoID, dx, dy):
    xid, yid = ReadMap.getXY(arucoID)
    xWalker = xid - dx
    yWalker = yid - dy

    return xWalker, yWalker

    
