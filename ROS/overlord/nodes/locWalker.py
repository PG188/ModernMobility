import ReadMap

def locWalker(arucoID, dx, dy):
    xid, yid, _ = ReadMap.getPose(arucoID)
    xWalker = xid - dx
    yWalker = yid - dy

    return xWalker, yWalker

    
