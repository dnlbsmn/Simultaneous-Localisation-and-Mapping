### =============================================== ###
# INITIALISATION

import cv2 as cv
import numpy as np
import a_star_lib as ASL
import time

### =============================================== ###
# MAIN CODE ALGORITHM

def run(image, start, end, ui_mode = 0):
	"""
	a_star_run takes a map and start and end points and finds the optimal path between them
	
	:image: a (binary) grayscale image of the map
	:start: the xy coordinates of the start point
	:end: the xy coordinates of the end point
	:ui_mode: 1 for showing ui, 0 for no ui, defaults to 0, no start and end input is needed if 1
	:return: returns a raster map of the optimal path
	"""
	
	ASL.init()
	ASL.load_map(image)
	
	if (ui_mode):
		ASL.initialise_points()
		start = ASL.start
		end = ASL.end
		
	ASL.open_cells = [start]
	ASL.path = [end]
	
	ASL.start = start
	ASL.end = end

	# Iterating through the A star algorithm
	tic = time.time()
	print("Planning a path...")

	current_point = ASL.start

	i = 0

	while (current_point != ASL.end):
	    current_point = ASL.open_cells[0]
	    
	    ASL.populate(current_point, ASL.end, ASL.maze_height, ASL.maze_width)
	    ASL.open_cells.sort(key = ASL.get_f)
	    
	    if (ui_mode & ((i % 40) == 0)):
	    	cv.imshow('map', ASL.maze)
	    	cv.waitKey(1)
	    	
	    i += 1

	ASL.find_path(ASL.end, ASL.maze_height, ASL.maze_width)
	ASL.path.reverse()

	ASL.maze[ASL.end[1]][ASL.end[0]] = ASL.PATH_COLOR

	print("Optimal path found.")
	toc = time.time()

	# A final display of victory
	if (ui_mode):
		cv.imshow('map', ASL.maze)
		cv.imshow('map', ASL.path_array)

		cv.imwrite("path_array.png", ASL.path_array)

		print("Elapsed Time: %.4f seconds" % (toc - tic))
		
		cv.waitKey(0)
		cv.destroyAllWindows()
		
	return ASL.path_array
