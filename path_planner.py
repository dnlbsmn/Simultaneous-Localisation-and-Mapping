import cv2 as cv
import numpy as np
import math

def create_vector(position, magnitude, heading):
	vector = [position, magnitude, heading]
	
	return vector

def change_to_heading(dx, dy):
	if ((dx == 0) & (dy == -1)): return 0
	if ((dx == 1) & (dy == -1)): return 1
	if ((dx == 1) & (dy == 0)): return 2
	if ((dx == 1) & (dy == 1)): return 3
	if ((dx == 0) & (dy == 1)): return 4
	if ((dx == -1) & (dy == 1)): return 5
	if ((dx == -1) & (dy == 0)): return 6
	if ((dx == -1) & (dy == -1)): return 7
	if ((dx == 0) & (dy == 0)): return 253

def path_to_vector(path_array, start):
	x = start[0]
	y = start[1]
	end_flag = True
	current_heading = 0

	vector = create_vector(start, 0, 0)
	while(end_flag):
		end_flag = False
		for_flag = False
		path_array[y][x] = 0

		for dx in [-1, 0, 1]:
			if (for_flag): break
			for dy in [-1, 0, 1]:
				if (path_array[y + dy][x + dx] == 255):
					end_flag = True
					current_heading = change_to_heading(dx, dy)

					cv.imshow("path", path)
					print(path_array[y][x])
					print("vectors: ", vectors)
					print("current_vector: ", vector)
					print("current_heading: ", current_heading)
					print("x,y ", x, " ", y)
					cv.waitKey(0)
					
					if (current_heading == vector[2]):
						y += dy
						x += dx
						vector[1] += math.sqrt(dx * dx + dy * dy)
						for_flag = True
						break
					else:
						vectors.append(vector)
						vector = create_vector((x, y), 0, current_heading)
						break
	vectors.append(vector)

### =========================================== ###
# MAIN CODE

cv.namedWindow("path", cv.WINDOW_NORMAL)
cv.resizeWindow("path", (1200, 960))

path = cv.imread("path_array.png")
path = cv.cvtColor(path, cv.COLOR_BGR2GRAY)

print(change_to_heading(1, 1))

start = [123, 118]
#path[start[1]][start[0]] = 0
vectors = []

cv.imshow("path", path)
cv.waitKey(0)

path_to_vector(path, start)

print(vectors)

cv.imshow("path", path)
cv.waitKey(0)

cv.destroyAllWindows()
