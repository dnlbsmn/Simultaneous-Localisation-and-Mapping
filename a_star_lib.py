import cv2 as cv
import numpy as np
import time

### =============================================== ###
# CONSTANTS

UNDEFINED = 0
UNPASSABLE = 1
OPEN = 2
CLOSED = 3
START = 4
PATH = 5

class_enum = ["Undefined ", "Unpassable", "Open      ", "Closed    ", "Start     ", "End       "] 

UNDEFINED_COLOR = [50, 50, 50]
UNPASSABLE_COLOR = [255, 255, 255]
CLOSED_COLOR = [77, 143, 84]
OPEN_COLOR = [34, 250, 248]
START_COLOR = [168, 109, 50]
PATH_COLOR = [163, 72, 227]

def initialise():
	global maze, maze_matrix, classes, path_array, h_costs, g_costs, maze_width, maze_height
	global open_cells, path, start, end

	start = []
	end = []

	maze_width = 0 
	maze_height = 0 

	maze = []
	maze_matrix = []
	classes = []
	path_array = []
	h_costs = []
	g_costs = []

	open_cells = []
	path = []

### =============================================== ###
# STRUCTURES

# Calculating an h cost for a single point
def gen_h(point, end):
	dx = abs(end[0] - point[0])
	dy = abs(end[1] - point[1])

	straight = abs(dx - dy)
	angle = min(dx, dy)

	return 10 * straight + 14 * angle

# Calculating a g costs of a point from a parent
def gen_g(child, parent, parent_g):
	if (child == parent):
		g = 0
	elif (child[0] == parent[0] or child[1] == parent[1]):
		g = 10
	else:
		g = 14
	
	return g + parent_g

### =============================================== ###close_cell
# ALGORITHM FUNCTIONS

# A function to find a general area that is optimal

def populate(parent, target, height, width):    
	for x in [parent[0] - 1, parent[0], parent[0] + 1]: 
		# Checking that values outside the array are not populated
		if ((x < 0) or (x >= width)):
			continue
	
		for y in [parent[1] - 1, parent[1], parent[1] + 1]: 
			# Checking that values outside the array are not populated
			if ((y < 0) or (y >= height)):
				continue
			elif (classes[y][x] == UNPASSABLE or classes[y][x] == CLOSED):
				continue

			# Testing if the new g value is lesser than the previous ones
			old_g = g_costs[y][x]
			new_g = gen_g([x, y], parent, g_costs[parent[1]][parent[0]])

			if (new_g < old_g or classes[y][x] == UNDEFINED):
				open_cell([x, y]) 

				# Updating values in the block
				g_costs[y][x] = new_g
				h_costs[y][x] = gen_h([x, y], target)

	close_cell(parent)

# A function for finding the path after the map has been populated

def find_path(end, height, width):
	g_array = [[end[0], end[1]]]

	while(g_array[0] != start):
		# Reseting for the next iteration
		[px, py] = g_array[0]
		g_array.clear()

		# Appending all possible next steps to a list
		for x in [px - 1, px, px + 1]: 
			# Checking that values outside the array are not populated
			if ((x < 0) or (x >= width)):
				continue

			for y in [py - 1, py, py + 1]:
				# Checking that values outside the array are not populated
				if ((y < 0) or (y >= height)):
					continue
				elif (classes[y][x] != CLOSED):
					continue

				g_array.append([x, y])

		g_array.sort(key = get_g)
		path_cell(g_array[0])
		
### =============================================== ###
# GUI FUNCTIONS

def select_points(event, x, y, flags, param):
	global start, end

	if event == cv.EVENT_LBUTTONDOWN:
		start = [x, y]
		print("Start:", start)
		maze[start[1]][start[0]] = START_COLOR
		cv.imshow('map', maze)

	if event == cv.EVENT_RBUTTONDOWN:
		end = [x, y]
		print("End:", end)
		maze[end[1]][end[0]] = START_COLOR
		cv.imshow('map', maze)

def display_info(event, x, y, flags, param):
	if event == cv.EVENT_LBUTTONDOWN:
		print(class_enum[int(classes[y][x])], " H_Cost:", h_costs[y][x], " G_Cost:", g_costs[y][x])

### =============================================== ###
# CELL LIST MANIPULATION

def get_f(point):
	return g_costs[point[1]][point[0]] + h_costs[point[1]][point[0]]

def get_g(point):
	return g_costs[point[1]][point[0]]

def open_cell(point):
	maze[point[1]][point[0]] = OPEN_COLOR
	classes[point[1]][point[0]] = OPEN

	open_cells.append([point[0], point[1]])

def close_cell(point):
	maze[point[1]][point[0]] = CLOSED_COLOR
	classes[point[1]][point[0]] = CLOSED

	open_cells.remove(point)

def path_cell(point):
	maze[point[1]][point[0]] = PATH_COLOR
	classes[point[1]][point[0]] = PATH
	path_array[point[1]][point[0]] = 255

	path.append(point)

### =============================================== ###
# MAIN CODE INITIALISATION

def load_map(maze_in):
	global maze, maze_matrix, maze_height, maze_width, classes, path_array, h_costs, g_costs
	
	maze_matrix = maze_in
	maze = cv.cvtColor(maze_matrix, cv.COLOR_GRAY2BGR)

	cv.namedWindow('map', cv.WINDOW_NORMAL)
	cv.resizeWindow('map', 1200, 1200)

	# Setting up the parallel matrices

	maze_height, maze_width = maze_matrix.shape
	classes = np.array(maze_matrix / 255, dtype=np.uint8)
	path_array = np.zeros([maze_height, maze_width], dtype=np.uint8)
	h_costs = np.zeros([maze_height, maze_width], dtype=np.uint16)
	g_costs = np.zeros([maze_height, maze_width], dtype=np.uint16)

def initialise_points():
	global start, end

	cv.setMouseCallback('map', select_points)

	start = [0, 0]
	end = [0, 0]

	print("Left click to select start, right click to select end.")

	cv.imshow('map', maze)
	cv.waitKey(0)

	cv.setMouseCallback('map', display_info)                                                                     

### =============================================== ###
# MAIN RUN SEQUENCE

def run(image, start_in = [], end_in = [], ui_mode = 0):
	global start, end, open_cells
	"""
	a_star_run takes a map and start and end points and finds the optimal path between them
	
	:image: a (binary) grayscale image of the map
	:start: the xy coordinates of the start point
	:end: the xy coordinates of the end point
	:ui_mode: 1 for showing ui, 0 for no ui, defaults to 0, no start and end input is needed if 1
	:return: returns a raster map of the optimal path
	"""
	
	initialise()
	load_map(image)
	
	if (ui_mode):
		initialise_points()
		start_in = start
		end_in = end
		
	print(start_in, end_in)

	open_cells = [start_in]
	path = [end_in]
	
	start = start_in
	end = end_in

	# Iterating through the A star algorithm
	tic = time.time()
	print("Planning a path...")

	current_point = start

	i = 0

	while (current_point != end):
		current_point = open_cells[0]
		
		populate(current_point, end, maze_height, maze_width)
		open_cells.sort(key = get_f)
		
		if (ui_mode & ((i % 40) == 0)):
			cv.imshow('map', maze)
			cv.waitKey(1)
			
		i += 1

	find_path(end, maze_height, maze_width)
	path.reverse()

	maze[end[1]][end[0]] = PATH_COLOR

	print("Optimal path found.")
	toc = time.time()

	# A final display of victory
	if (ui_mode):
		cv.imshow('map', maze)
		cv.imshow('map', path_array)

		cv.imwrite("path_array.png", path_array)

		print("Elapsed Time: %.4f seconds" % (toc - tic))
		
		cv.waitKey(0)
		cv.destroyAllWindows()
		
	return path_array, start


#maze = cv.imread("cleaned.png")
#maze = cv.cvtColor(maze, cv.COLOR_BGR2GRAY)

#run(maze, ui_mode = 1)