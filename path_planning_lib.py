import cv2 as cv
import numpy as np
import math

# creates a vector object
def create_vector(position, magnitude, heading):
	vector = [position, magnitude, heading]
	return vector

# encodes vector headings to numbers (clockwise)
def change_to_heading(dx, dy):
	if ((dx == 0) & (dy == -1)): return -90
	if ((dx == 1) & (dy == -1)): return -45
	if ((dx == 1) & (dy == 0)): return 0
	if ((dx == 1) & (dy == 1)): return 45
	if ((dx == 0) & (dy == 1)): return 90
	if ((dx == -1) & (dy == 1)): return 135
	if ((dx == -1) & (dy == 0)): return 180
	if ((dx == -1) & (dy == -1)): return -135
	print("Heading not found")
	if ((dx == 0) & (dy == 0)): return 253

# follows a continuous grayscale path from start to end generating an array of vector objects
def path_to_vector(path_array, start, ui_mode):
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

					if (current_heading == vector[2]):
						y += dy
						x += dx
						vector[1] += math.sqrt(dx * dx + dy * dy)
						loop_flag = True
						break
					else:
						if (ui_mode == 1):
							cv.imshow("path", path_array)
							print("current_vector: ", vector)
							print("current_heading: ", current_heading)
							print("x,y ", x, " ", y)
							#cv.waitKey(0)
						
						vectors.append(vector)
						vector = create_vector((x, y), 0, current_heading)
						break

	vectors.append(vector)
	if (ui_mode == 1):
		cv.imshow("path", path_array)
		print("current_vector: ", vector)
		print("current_heading: ", current_heading)
		print("x,y ", x, " ", y)
		print(vectors)
		cv.waitKey(0)
		cv.destroyAllWindows()
	return vectors

# intermediate function for ui initilise
def convert_path_to_vectors(path, start, ui_mode): # map array, start[x,y], ui_mode
	if (ui_mode == 1):
		cv.namedWindow("path", cv.WINDOW_NORMAL)
		cv.resizeWindow("path", (1200, 960))
		cv.imshow("path", path)
		cv.waitKey(0)
	
	vectors = path_to_vector(path, start, ui_mode)
	return vectors

def merge_vectors(vectors, path, path_ui, ui_mode): 
	print(vectors)
	print("Merged vectors:")
	vectors_merged = []
	end = [0, 0]
	clear_flag = False
	for v_index in range(len(vectors)):
		# If this vector was merged on last iteration, skip to next vector
		if (clear_flag): 
			clear_flag = False
		elif (v_index == len(vectors)-1):
			vector_rads = create_vector(vectors[v_index][0], vectors[v_index][1], vectors[v_index][2]) #-2)*math.pi/4)
			vectors_merged.append(vector_rads)
		elif (vectors[v_index][1] != 0):
			end[0] = vectors[v_index+1][0][0] + (vectors[v_index+1][1] * math.cos(vectors[v_index+1][2]*(math.pi/180))) #-2)*(math.pi/4))) 
			end[1] = vectors[v_index+1][0][1] + (vectors[v_index+1][1] * math.sin(vectors[v_index+1][2]*(math.pi/180))) #-2)*(math.pi/4)))
			x_diff = end[0] - vectors[v_index][0][0]
			y_diff = end[1] - vectors[v_index][0][1]
			print(end)
			
			try: 
				angle = (180/math.pi)*math.atan(y_diff/x_diff)
				if (x_diff < 0): angle += 180# math.pi
				# 90 degrees?

			except:
				print("angle be funky")
				if (vectors[v_index][2] == 0):
					angle = -90
				elif (vectors[v_index][2] == 4):
					angle = 90
				print(vectors[v_index])
			
			mag = math.sqrt(x_diff*x_diff + y_diff*y_diff)
			clear_flag = True
			prev_y = vectors[v_index][0][1]
			step = 1
			obs_flag = False
			for i in range(0, int(x_diff), int(x_diff/abs(x_diff))):
				y = vectors[v_index][0][1] + i*math.tan(angle*(math.pi/180))
				x = vectors[v_index][0][0] + i
				
				if (y<prev_y): step = -1

				for j in range(0, int(y - prev_y) + step, step):
					if (path[int(y+j)][int(x)] == 255): # obstacle????
						clear_flag = False
						print("obstacle")
						cv.waitKey(1) #temp
						obs_flag = True
						break
					if (ui_mode == 1):
						path_ui[int(y+j)][int(x)] = 180
						
				if (obs_flag): break
				prev_y = y


			if (clear_flag):
				vectors_merged.append(create_vector(vectors[v_index][0], mag, angle))
			else:
				vector_rads = create_vector(vectors[v_index][0], vectors[v_index][1], vectors[v_index][2])# -2)*math.pi/4)
				vectors_merged.append(vector_rads)
			if (ui_mode == 1):
				path_ui[vectors[v_index][0][1]][vectors[v_index][0][0]] = 60
	
	if (ui_mode == 1):
		cv.namedWindow("path", cv.WINDOW_NORMAL)
		cv.resizeWindow("path", (1200, 960))
		cv.imshow("path", path_ui)
		cv.waitKey(0)
	return vectors_merged
			



