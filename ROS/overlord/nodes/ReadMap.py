#ReadMap.py

"""
This file reads in the data from 'map_config.json' and
plots it to visualize what the data currently represents
"""

import json
import numpy as np
import matplotlib.pyplot as plt

def readMap():
    map_data = open("map_config.json", "r")
    map_str = map_data.read()
    map_data.close()

    print ("[ReadMap.py]:readMap():Data read from map_config.json:")
    print (map_str)
    map_dict = json.loads(map_str)
    xcoords = []
    ycoords = []

    for k,v in map_dict.iteritems():
        xcoords.append(v["x"])
        ycoords.append(v["y"])

    return xcoords, ycoords

def drawMap(xcoords, ycoords, invertX = False, invertY = False):
    #Some settings
    fig = plt.figure()
    fig.canvas.set_window_title("ArUco Marker Locations")
    plt.grid(True)

    #Invert the axes if required
    if invertX: plt.gca().invert_xaxis()
    if invertY: plt.gca().invert_yaxis()

    #Draw the map
    plt.scatter(xcoords, ycoords)
    plt.show()

if __name__ == "__main__":
    x,y = readMap()
    drawMap(x,y)
