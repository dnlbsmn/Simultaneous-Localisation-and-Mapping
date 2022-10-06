### =============================================== ###
# INITIALISATION

import cv2 as cv
import numpy as np
import a_star_lib as ASL
import time

### =============================================== ###
# MAIN CODE ALGORITHM

image_path = input("What maze file do you want to use?: ")

ASL.init()

ASL.load_map(image_path)
ASL.initialise_points()

print(ASL.start)
print(ASL.end)

ASL.open_cells = [ASL.start]
ASL.path = [ASL.end]

# Iterating through the A star algorithm
tic = time.time()
print("Processing...")

current_point = ASL.start

while (current_point != ASL.end):
    current_point = ASL.open_cells[0]
    
    ASL.populate(current_point, ASL.end, ASL.maze_height, ASL.maze_width)
    ASL.open_cells.sort(key = ASL.get_f)
    
    cv.imshow('map', ASL.maze)
    cv.waitKey(1)

ASL.find_path(ASL.end, ASL.maze_height, ASL.maze_width)
ASL.path.reverse()

ASL.maze[ASL.end[1]][ASL.end[0]] = ASL.PATH_COLOR

print("Optimal path found.")
toc = time.time()

# A final display of victory
cv.imshow('map', ASL.maze)
cv.imshow('map', ASL.path_array)

cv.imwrite("path_array.png", ASL.path_array)

print("Elapsed Time: %.4f seconds" % (toc - tic))

cv.waitKey(0)
cv.destroyAllWindows()
