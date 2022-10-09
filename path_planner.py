import cv2 as cv
import numpy as np
import math

# creates a vector object
def create_vector(position, magnitude, heading):
	vector = [position, magnitude, heading]
	return vector

# encodes vector headings to numbers (clockwise)
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

# follows a continuous grayscale path from start to end generating an array of vector objects
def path_to_vector(path_array, start, UI_mode):
	x = start[0]
	y = start[1]
	end_flag = True
	current_heading = 0
	vectors = []
	vector = create_vector(start, 0, 0)
	while(end_flag):
		end_flag = False
		loop_flag = False
		path_array[y][x] = 0

		for dx in [-1, 0, 1]:
			if (loop_flag): break
			for dy in [-1, 0, 1]:
				if (path_array[y + dy][x + dx] == 255):
					end_flag = True
					current_heading = change_to_heading(dx, dy)
					if (current_heading == 253):
						print("ERROR")
					
					if (current_heading == vector[2]):
						y += dy
						x += dx
						vector[1] += math.sqrt(dx * dx + dy * dy)
						loop_flag = True
						break
					else:
						if (UI_mode == 1):
							cv.imshow("path", path_array)
							print("current_vector: ", vector)
							print("current_heading: ", current_heading)
							print("x,y ", x, " ", y)
							cv.waitKey(0)
						
						vectors.append(vector)
						vector = create_vector((x, y), 0, current_heading)
						break

	vectors.append(vector)
	if (UI_mode == 1):
		cv.imshow("path", path_array)
		print("current_vector: ", vector)
		print("current_heading: ", current_heading)
		print("x,y ", x, " ", y)
		print(vectors)
		cv.waitKey(0)
		cv.destroyAllWindows()
	return vectors

# intermediate function for ui initilise
def convert_path_to_vectors(path, start, UI_mode): # map array, start[x,y], UI_mode
	if (UI_mode == 1):
		cv.namedWindow("path", cv.WINDOW_NORMAL)
		cv.resizeWindow("path", (1200, 960))
		cv.imshow("path", path)
		cv.waitKey(0)
	
	vectors = path_to_vector(path, start, UI_mode)
	return vectors

def merge_vectors(vectors, path): 
	print(vectors)
	vectors_merged = []
	end = [0, 0]
	clear_flag = False
	for v_index in range(len(vectors)- 1):
		print(v_index)
		# If this vector was merged on last iteration, skip to next vector
		if (clear_flag): 
			print(skipped)
			clear_flag = False
			break


		if (vectors[v_index][1] != 0):
			end[0] = vectors[v_index+1][0][0] + (vectors[v_index+1][1] * math.sin(vectors[v_index+1][2]*(math.pi/4))) 
			end[1] = vectors[v_index+1][0][1] + (vectors[v_index+1][1] * math.cos(vectors[v_index+1][2]*(math.pi/4)))
			x_diff = end[0] - vectors[v_index][0][0]
			y_diff = end[1] - vectors[v_index][0][1]
			angle = math.asin(y_diff/x_diff)
			mag = math.sqrt(x_diff*x_diff + y_diff*y_diff)
			clear_flag = True
			print(x_diff)
			for i in range(int(x_diff)):
				y = int(vectors[v_index][0][1] + i*math.sin(angle))
				x = vectors[v_index][0][1] + i
				if (path[y][x] == 255): # obstacle????
					clear_flag = False
					break
			if (clear_flag):
				vectors_merged.append(create_vector(vectors[v_index][0], mag, angle))
	print(vectors_merged)
			


