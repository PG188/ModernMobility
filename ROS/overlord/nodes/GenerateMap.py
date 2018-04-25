import ReadMap

#Generates map and writes it to config file
ReadMap._writeMap(ReadMap._generateMap())

#Reads map config file and visualizes it on a plot
const_map, config_map = ReadMap._readMap()
ReadMap._drawMap(config_map)
