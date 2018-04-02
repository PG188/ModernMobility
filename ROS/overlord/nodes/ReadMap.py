#ReadMap.py

"""
This file reads in the data from 'map_config.json' and
plots it to visualize what the data currently represents

The .json file structure looks like the following:
{
"Data":
    {
    "ID":{"x":num,"y":num,"theta":num},...
    },
"Parameters":
    {
    "room_length": num,
    "room_width": num,
    "max_spacing": num,
    "x_spacing": num,
    "y_spacing": num
    }
}       
"""

import json
import numpy as np
import matplotlib.pyplot as plt
import sys


MAP_CONST_FILE = "map_constants.json"   #"filename.json"
MAP_CONFIG_FILE = "map_config.json"     #"filename.json"

def _readMap():
    const_dict = {}
    config_dict = {}
    try:
        map_const = open(MAP_CONST_FILE, "r")
        const_str = map_const.read()
        map_const.close()
        
        map_config = open(MAP_CONFIG_FILE, "r")
        config_str = map_config.read()
        map_config.close()

        const_dict = json.loads(const_str)
        config_dict = json.loads(config_str)
        
    except Exception as e:
        print ("\n[ReadMap.py]:readMap():\n\tError while reading map_config.json:\n\t%s\n" % str(e))

    finally:
        return const_dict, config_dict

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
            print ("\n\nHit enter to reuse the already loaded values.") 
            print ("Please specify values in meters:\n")
            py_ver = sys.version_info[0]
            if (py_ver == 2):
                max_spacing = raw_input("What is the max spacing between ArUco markers allowed? ")
                width = raw_input("What is the width (x-axis) of the room? ")
                length = raw_input("What is the length (y-axis) of the room? ")
            elif (py_ver == 3):
                max_spacing = input("What is the max spacing between ArUco markers allowed? ")
                width = input("What is the width (x-axis) of the room? ")
                length = input("What is the length (y-axis) of the room? ")
            else:
                print("\n[ReadMap.py]:_drawMap():\nMust be using python version 2 or 3\n")

            needInputs = False
            
        except:
            print("\nError with input types, please make sure they are numberic!\n")
            needInputs = True

    #Load in current data
    const_dict, config_dict = _readMap()
    if (max_spacing == ''): max_spacing = config_dict["Parameters"]["max_spacing"]
    if (width == ''): width = config_dict["Parameters"]["room_width"]
    if (length == ''): length = config_dict["Parameters"]["room_length"]  

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

    #Added constants to config file first    
    print("\nArUco Marker Locations:")
    print("\nID\t|x\t|y\t|theta")
    print("------------------------------")
    offset = 0
    constants = 0
    config_dict_data = {}
    if (py_ver == 2):
        for ID,pose in const_dict["Data"].iteritems():
            constants += 1
            offset = max(offset,int(ID)+1)
            config_dict_data[ID] = {"x":pose["x"],
                                    "y":pose["y"],
                                    "theta":pose["theta"]}
            print ("%s\t|%3.3f\t|%3.3f\t|%3.3f"
                   % (ID, pose["x"], pose["y"], pose["theta"]))
    elif (py_ver == 3):
        for ID,pose in const_dict["Data"].items():
            constants += 1
            offset = max(offset,int(ID)+1)
            config_dict_data[ID] = {"x":pose["x"],
                                    "y":pose["y"],
                                    "theta":pose["theta"]}
            print ("%s\t|%3.3f\t|%3.3f\t|%3.3f"
                   % (ID, pose["x"], pose["y"], pose["theta"]))
    else:
        print("\n[ReadMap.py]:_drawMap():\nMust be using python version 2 or 3\n")
    

    #Generate the rest of the points for the config file
    print("------------------------------")
    for j in range(y):
        for i in range(x):
            ID = i + (j * x) + offset
            x_coord = (i + 1) * x_spacing
            y_coord = (j + 1) * y_spacing
            print ("%d\t|%3.3f\t|%3.3f\t|%3.3f" % (ID, x_coord, y_coord, 0.000))
            config_dict_data[ID] = {"x":x_coord, "y":y_coord, "theta":0.000}

    print("""
Summary:
    ArUco markers used:     %d
    Room (width x length):  %3.3fm x %3.3fm   
    Spacing used (Max allowed = %3.3fm):
        x direction: %3.3fm
        y direction: %3.3fm"""
          % (x*y+constants,width, length, max_spacing, x_spacing, y_spacing))

    #Use new data to modify the map data and return it
    config_dict["Data"] = config_dict_data
    config_dict["Parameters"] = {"max_spacing":max_spacing,
                              "x_spacing":x_spacing,
                              "y_spacing":y_spacing,
                              "room_width":width,
                              "room_length":length}

    return config_dict

def _writeMap(map_dict):
    map_file = open(MAP_CONFIG_FILE, "w")
    map_str = json.dumps(map_dict)
    map_file.write(map_str)
    map_file.close()

def getPose(ID):
    #Returns the x, y position of the marker given its ID number
    const_map, config_map = _readMap()
    x = config_map["Data"][str(ID)]["x"]
    y = config_map["Data"][str(ID)]["y"]
    theta = config_map["Data"][str(ID)]["theta"]
    return x, y, theta

def getConstPose(name):
    const_map, config_map = _readMap()
    const_dict_data = const_map["Data"]
    py_ver = sys.version_info[0]
    if (py_ver == 2):
        for k,v in const_dict_data.iteritems():
            if name == v["name"]:
                return v["x"],v["y"],v["theta"]
    elif (py_ver == 3):
        for k,v in const_dict_data.items():
            if name == v["name"]:
                return v["x"],v["y"],v["theta"]
    else:
        print("\n[ReadMap.py]:_drawMap():\nMust be using python version 2 or 3\n")

    

    
if __name__ == "__main__":
    #Generates map and writes it to config file
    _writeMap(_generateMap())

    #Reads map config file and visualizes it on a plot
    const_map, config_map = _readMap()
    _drawMap(config_map)
