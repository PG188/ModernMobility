#ReadMap.py

"""
This file reads in the data from 'map_config.json' and
plots it to visualize what the data currently represents
"""

import json
import numpy as np
import matplotlib.pyplot as plt

def readMap():
    xcoords = []
    ycoords = []
    try:
        map_data = open("map_config.json", "r")
        map_str = map_data.read()
        map_data.close()

        map_dict = json.loads(map_str)
        map_dict = map_dict["Data"]
        
        for k,v in map_dict.iteritems():
            xcoords.append(v["x"])
            ycoords.append(v["y"])

    except Exception as e:
        print ("""\n[ReadMap.py]:readMap():\n\tError while reading
                map_config.json:\n\t%s\n""" % str(e))

    finally:
        return xcoords, ycoords

def _drawMap(xcoords, ycoords, invertX = False, invertY = False):
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
    x, y = readMap()
    _drawMap(x, y)
