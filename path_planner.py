import cv2 as cv
import numpy as np
import math

def create_vector(position, magnitude, heading):
	vector = [position, magnitude, heading]
	
	return vector

def change_to_heading(dx, dy):
	if (dx == 0 & dy == -1): return 0
	if (dx == 1 & dy == -1): return 1
	if (dx == 1 & dy == 0): return 2
	if (dx == 1 & dy == 1): return 3
	if (dx == 0 & dy == 1): return 4
	if (dx == -1 & dy == 1): return 5
	if (dx == -1 & dy == 0): return 6
	if (dx == -1 & dy == -1): return 7

def path_to_vector(path_array, start):
	x = start[0]
	y = start[1]

	vector = create_vector(start, 0, 4)
	while(True):
		end_flag = False
		for dx in [-1, 0, 1]:
			for dy in [-1, 0, 1]:
				if (path_array[y + dy][x + dx] == 255):
					end_flag = True
					path_array[y][x] = 0	
					y += dy
					x += dx
					current_heading = change_to_heading(dx, dy)
					
					if (current_heading == vector[2]):
						vector[1] += math.sqrt(dx * dx + dy * dy)
					else:
						vectors.append(vector)
						vector = create_vector((x, y), 0, current_heading)
						break
		if (end_flag == False):
			break

### =========================================== ###
# MAIN CODE

cv.namedWindow("path", cv.WINDOW_NORMAL)
cv.resizeWindow("path", (1200, 960))

path = cv.imread("path_array.png")
path = cv.cvtColor(path, cv.COLOR_BGR2GRAY)

start = [58, 50]
vectors = []

path_to_vector(path, start)

print(vectors)

cv.imshow("path", path)
cv.waitKey(0)

cv.destroyAllWindows()
