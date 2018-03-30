#ReadMap.py

"""
This file reads in the data from 'map_config.json' and
plots it to visualize what the data currently represents
"""

import json
import numpy as np
import matplotlib.pyplot as plt
import sys

MAP_CONFIG_FILE = "map_config.json" #filename.extension (Should be a .json file)

def readMap():
    map_dict = {}
    try:
        map_file = open(MAP_CONFIG_FILE, "r")
        map_str = map_file.read()
        map_file.close()

        map_dict = json.loads(map_str)
        

    except Exception as e:
        print ("\n[ReadMap.py]:readMap():\n\tError while reading map_config.json:\n\t%s\n" % str(e))

    finally:
        return map_dict

def _drawMap(map_dict, invertX = False, invertY = False):
    #Some settings
    fig = plt.figure()
    ax = fig.add_subplot(111)
    fig.canvas.set_window_title("ArUco Marker Locations")
    plt.grid(True)

    #Invert the axes if required
    if invertX: plt.gca().invert_xaxis()
    if invertY: plt.gca().invert_yaxis()

    #Draw the walls
    width = map_dict["Parameters"]["room_width"]
    length = map_dict["Parameters"]["room_length"]
    rect = plt.Rectangle((0,0),width,length,facecolor='none',edgecolor='r')
    ax.add_patch(rect)
    
    #Draw the map
    x_coords = []
    y_coords = []
    py_ver = sys.version_info[0]
    if (py_ver == 2):
        for k,v in map_dict["Data"].iteritems():
            x_coords.append(v["x"])
            y_coords.append(v["y"])
    elif (py_ver == 3):
        for k,v in map_dict["Data"].items():
            x_coords.append(v["x"])
            y_coords.append(v["y"])
    else:
        print("\n[ReadMap.py]:_drawMap():\nMust be using python version 2 or 3\n")
    
    plt.scatter(x_coords, y_coords)
    plt.show()

def _generateMap():
    needInputs = True
    max_spacing = 0
    length = 0
    width = 0

    #Get input parameters
    while(needInputs):
        try:
            print ("Hit enter to reuse the already loaded values.") 
            print ("Please specify values in meters:")
            max_spacing = input("What is the max spacing between ArUco markers allowed? ")
            width = input("What is the width (x-axis) of the room? ")
            length = input("What is the length (y-axis) of the room? ")
            
            needInputs = False
            
        except:
            print("\nError with input types, please make sure they are numberic!\n")
            needInputs = True

    #Load in current data
    map_dict = readMap()
    map_dict_data = {}
    if (max_spacing == ''): max_spacing = map_dict["Parameters"]["max_spacing"]
    if (width == ''): width = map_dict["Parameters"]["room_width"]
    if (length == ''): length = map_dict["Parameters"]["room_length"]  

    #Make values float
    max_spacing = float(max_spacing)
    width = float(width)
    length = float(length)

    #Start with spacing being equal to the max allowed
    x_spacing = max_spacing
    y_spacing = max_spacing

    #Cast to int because we can't have a decimal number of spaces between markers
    x = int(width/x_spacing) - 1    #Numbers of markers along the width
    y = int(length/y_spacing) - 1   #Numbers of markers along the length
        
    #Find the numbers of ArUco markers needed
    xSpaceTooBig = True
    ySpaceTooBig = True
    while (xSpaceTooBig or ySpaceTooBig):
        num_of_markers = x * y

        #Find actual spacing to be used
        x_spacing = width/(x+1)
        y_spacing = length/(y+1)

        #If spacing is too big we need to add another markers (space)
        xSpaceTooBig = x_spacing > max_spacing
        ySpaceTooBig = y_spacing > max_spacing
        if xSpaceTooBig: x += 1
        if ySpaceTooBig: y += 1

    #Modify the map config file
    print("\nArUco Marker Locations:\n\nID\t|x\t|y\n-----------------------")
    for j in range(y):
        for i in range(x):
            ID = i+(j*x)
            x_coord = (i+1)*x_spacing
            y_coord = (j+1)*y_spacing
            print ("%d\t|%3.3f\t|%3.3f" % (ID, x_coord, y_coord))
            map_dict_data[ID] = {"x":x_coord, "y":y_coord}

    print("""
Summary:
ArUco markers used: %d
Room dimensions (width x length): %3.3fm x %3.3fm
Max allowed spacing: %3.3fm
Spacing used:
    x direction: %3.3fm
    y direction: %3.3fm"""
          % (x*y,width, length, max_spacing, x_spacing, y_spacing))

    #Use new data to modify the map data and return it
    map_dict["Data"] = map_dict_data
    map_dict["Parameters"] = {"max_spacing":max_spacing,
                              "x_spacing":x_spacing,
                              "y_spacing":y_spacing,
                              "room_width":width,
                              "room_length":length}

    return map_dict

def _writeMap(map_dict):
    map_file = open(MAP_CONFIG_FILE, "w")
    map_str = json.dumps(map_dict)
    map_file.write(map_str)
    map_file.close()

    
if __name__ == "__main__":
    _writeMap(_generateMap())   #Generates map and writes it to config file
    _drawMap(readMap())         #Reads map config file and visualizes it on a plot
