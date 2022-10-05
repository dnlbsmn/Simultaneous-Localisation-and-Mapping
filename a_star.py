### =============================================== ###
# INITIALISATION

import cv2 as cv
import numpy as np
import time
from a_star_lib import *
import image_preprocessing
import time

### =============================================== ###
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
    global start
    global end
    
    if event == cv.EVENT_LBUTTONDOWN:
        start = [x, y]
        print("Start:", start)
    if event == cv.EVENT_RBUTTONDOWN:
        end = [x, y]
        print("End:", end)

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

    path.append(point)

### =============================================== ###
# MAIN CODE INITIALISATION

# Loading the image

#image_path = input("What maze file do you want to use?: ")
image_path = "my_map_bw.bmp"
image_preprocessing.process(image_path)

# Importing the images

cv.namedWindow('map', cv.WINDOW_NORMAL)
cv.resizeWindow('map', 1200, 1200)

maze_matrix = cv.imread("processed_maze.png", cv.IMREAD_GRAYSCALE)
maze = cv.imread("processed_maze.png")

# Setting up the parallel matrices

maze_height, maze_width = maze_matrix.shape
classes = np.array(maze_matrix / 255, dtype=np.uint8)
h_costs = np.zeros([maze_height, maze_width], dtype=np.uint16)
g_costs = np.zeros([maze_height, maze_width], dtype=np.uint16)

# Assigning points

cv.setMouseCallback('map', select_points)

start = [0, 0]
end = [0, 0]

print("Left click to select start, right click to select end.")
cv.imshow('map', maze)
cv.waitKey(0)

# Drawing the start and end point

cv.setMouseCallback('map', display_info)

maze[start[1]][start[0]] = OPEN_COLOR
classes[start[1]][start[0]] = OPEN

maze[end[1]][end[0]] = PATH_COLOR
classes[end[1]][end[0]] = UNDEFINED

open_cells = [start]
path = [end]

### =============================================== ###
# MAIN CODE ALGORITHM

tic = time.time()

print("Processing...")

current_point = start

while (current_point != end):
    current_point = open_cells[0]
    
    populate(current_point, end, maze_height, maze_width)
    open_cells.sort(key = get_f)
    
##    cv.imshow('map', maze)
##    cv.waitKey(1)

##print("Maze:\n", classes, "\n")
##print("H Costs:\n", h_costs, "\n")
##print("G Costs:\n", g_costs, "\n")

find_path(end, maze_height, maze_width)

maze[end[1]][end[0]] = PATH_COLOR

cv.imshow('map', maze)

path.reverse()
#print("Path:\n", path, "\n")

toc = time.time()

print("Elapsed Time: %.4f seconds" % (toc - tic))

cv.waitKey(0)
cv.destroyAllWindows()

### =============================================== ###
# NOTES

# f-cost is the sum of the following
# g-cost is the distance from the start
# h-cost is the distance from the target
