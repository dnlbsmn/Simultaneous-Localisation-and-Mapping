### ===================================== ###
# IMPORTING EVERYTHING

from time import sleep
import math
from turtle import heading
import numpy as np
import cv2 as cv

import slamBotHD as sb
import a_star_lib as al
import path_planning_lib as pp
import depth_rectification_lib as dr
import bot_move_lib as bm
import landmark_generation_lib as lg
import particle_filtering_lib as pf
import map_generation_lib as mg

### ===================================== ###
# DEFINING FUNCTIONS

# Displays the observed and global maps
def display_maps():
	global foundational_landmarks

	display_landmarks(foundational_landmarks, color = 196)

	window = np.concatenate((dr.global_map, dr.global_preview), axis=1)
	cv.imshow("Map generation", window) 
	cv.waitKey(1)	

# Reads the slit of the depth sensor
def read_slit():
	depth_array = sb.imgDepth
	depth_slice = dr.convert_to_slice(depth_array)

	return depth_slice
	
# Using a given slice writes landmarks to an array
def read_landmarks(slyce_1d, ui_mode = 1): 
	dr.local_map = np.zeros((601, 601), dtype = np.uint8)

	dr.project_slit(slyce_1d)

	# Take the projected slit and return the relative coordinates of the landmarks
	view = lg.clean_local_map(dr.local_map)
	observed_landmarks = lg.observe_landmarks(view, ui_mode)

	return observed_landmarks

# Renders the bot with coordinates on a given display
def render_bot(x1, y1, heading, image):
	heading = heading * math.pi / 180

	x2 = x1 + int(17 * math.sin(heading))
	y2 = y1 + int(17 * math.cos(heading))

	cv.circle(image, (x1, y1), 17, 127, 1)
	cv.line(image, (x1, y1), (x2, y2), 127, 1)

# Displays a set of landmarks on the global map
def display_landmarks(rectified_landmarks, x = 300, y = 300, color = 128):
	cv.circle(dr.global_preview, (x, y), 7, 255, 1)

	for landmark in rectified_landmarks:
		cv.circle(dr.global_preview, (x + landmark[0], y + landmark[1]), 5, color, 1)

		#display_maps()
		cv.waitKey(10)

# Executes the actions for when the map is being scanned
def scan_action(key):
	global x, y, slyce_list

	# Display the global and local map
	if (key == ord('q')):
		#for i in range(len(slyce_list)):
		#	dr.interpret_slit(dr.global_map, slyce_list[i][0], x, y, slyce_list[i][1] + bm.heading_offset, 255)
		display_maps()
		return 1

	# Delete scan without solidifying
	elif (key == ord('r')):
		return 1

	# Correct robot position/heading
	elif (key == ord('w')):
		y -= 1
	elif (key == ord('a')):
		x -= 1
	elif (key == ord('s')):
		y += 1
	elif (key == ord('d')):
		x += 1
	elif (key == ord('z')):
		bm.heading_offset += 1
	elif (key == ord('x')):
		bm.heading_offset -= 1

	# Display perceived map
	dr.global_preview = dr.global_map.copy()
	for i in range(len(slyce_list)):
		dr.interpret_slit(dr.global_preview, slyce_list[i][0], x, y, slyce_list[i][1] + bm.heading_offset, 127)
		
# Returns the distance of a point from the origin
def origin_distance(point):
	return math.dist(point, (0, 0))

# Run a full spin and match it to a global map
def full_spin():
	global x, y

	slyce_list = []
	landmark_list = []

	slyce_1d = read_slit()
	slyce_list.append([slyce_1d, sb.imuYaw])

	observed_landmarks = read_landmarks(slyce_1d)
	landmark_list.append([observed_landmarks, sb.imuYaw])

	display_maps()

	for i in range(6):
		# Rotate and wait until things have settled
		bm.relative_rotate(45)
		cv.waitKey(3000)

		# Records the observation and direction of the bot
		slyce_1d = read_slit()
		slyce_list.append([slyce_1d, sb.imuYaw])

		# Records the landmarks and direction of the bot
		observed_landmarks = read_landmarks(slyce_1d)
		landmark_list.append([observed_landmarks, sb.imuYaw])

		display_maps()

		# Displaying the movement
		dr.global_preview = dr.global_map.copy()
		render_bot(x, y, bm.get_heading(), dr.global_preview)

		display_maps()

	rectified_landmarks = mg.map_landmarks(landmark_list, bm.heading_offset)
	display_landmarks(rectified_landmarks, x, y)

	shift = mg.match_corners(foundational_landmarks, rectified_landmarks)
	print(shift)
	x = 300 + shift[0]
	y = 300 + shift[1]

	for i in range(len(slyce_list)):
		dr.interpret_slit(dr.global_map, slyce_list[i][0], x, y, slyce_list[i][1] + bm.heading_offset, 255)

	render_bot(x, y, bm.get_heading(), dr.global_preview)
	display_landmarks(rectified_landmarks, x, y)
	display_maps()

# Pathfinds from the current position to a given end position
def path_find(end, ui_mode = 1):
	global traverse_map, x, y
	# Generating areas where the bot can traverse and running A*
	cleaned_map = lg.clean_traverse_map(dr.global_map)
	traverse_map = cv.dilate(cleaned_map, np.ones((22, 22)))
	path, start = al.run(traverse_map, [int(x), int(y)], end, ui_mode = ui_mode)

	# Converting the basic A* vectors to something better
	path_4_ui = cv.addWeighted(traverse_map, 0.3, path, 1, 0)
	vectors = pp.convert_path_to_vectors(path, start, ui_mode=ui_mode)
	render_bot(x, y, bm.get_heading(), path_4_ui)
	vectors_merged = pp.merge_vectors(vectors, traverse_map, path_4_ui, ui_mode=ui_mode)

	# Convert to correct angles and execute movements
	for vector in vectors_merged:
		vector[2] -= 90

		bm.relative_rotate(bm.angle_difference(-vector[2], bm.get_heading()))
		print(bm.angle_difference(-vector[2], bm.get_heading()))
		cv.waitKey(1000)

		x, y = bm.relative_move(vector[1], x, y)

		# Displaying the movement
		render_bot(x, y, bm.get_heading(), path_4_ui)
		cv.imshow("Path planning", path_4_ui)
		cv.waitKey(1000)

# Generate an appropriate movement towards a corner
def phantom_corner(corner):
	global traverse_map

	for i in range(30):
		x_offset = (corner[0]) * (0.7 - i/100)
		y_offset = (corner[1]) * (0.7 - i/100)
		p_corner = [int(origin[0] + x_offset), int(origin[1] + y_offset)]

		#if (traverse_map[p_corner[1]][p_corner[0]] == 0):
		return p_corner

### ===================================== ###
# INITIALISATION	
	
dr.initialise_matrices()
dr.initialise_tan()
mg.initialise()

sb.startUp()
sb.readCoreData()
sleep(2)

print("Battery voltage:", sb.botBatt, "V")

origin = [300, 300]

print(phantom_corner([-57, -200]))

x = origin[0]
y = origin[1]
bm.heading_offset = 0
foundational_landmarks = []

# How do you want to run the program?
# 1 = Generate the map 
# 2 = Find Yourself in a known map
# 3 = Manual map generation
# 4 = Just see the bot input
# 5 = Manual generate + Particle filter

run_mode = 2

### ===================================== ###
# ACTUAL MAIN

########################
# Map Generation Stage #
########################

if ((run_mode == 1)):
	# spin to obtain first corner map
	slyce_list = []
	landmark_list = []

	slyce_1d = read_slit()
	dr.interpret_slit(dr.global_preview, slyce_1d, x, y, bm.get_heading(), 255)
	slyce_list.append([slyce_1d, sb.imuYaw])

	observed_landmarks = read_landmarks(slyce_1d)
	landmark_list.append([observed_landmarks, sb.imuYaw])

	display_maps()

	# Doing a full spin to get the foundational landmarks
	for i in range(7):
		# Rotate and wait until things have settled
		bm.relative_rotate(45)
		cv.waitKey(3000)

		# Records the observation and direction of the bot
		slyce_1d = read_slit()
		slyce_list.append([slyce_1d, sb.imuYaw])

		# Records the landmarks and direction of the bot
		observed_landmarks = read_landmarks(slyce_1d)
		landmark_list.append([observed_landmarks, sb.imuYaw])

		# Displaying the movement
		dr.global_preview = dr.global_map.copy()
		render_bot(x, y, bm.get_heading(), dr.global_preview)

		foundational_landmarks = mg.map_landmarks(landmark_list, bm.heading_offset)
		display_landmarks(foundational_landmarks, color = 196)

		for i in range(len(slyce_list)):
			dr.interpret_slit(dr.global_preview, slyce_list[i][0], x, y, slyce_list[i][1] + bm.heading_offset, 255)

		display_maps()

	for i in range(len(slyce_list)):
		dr.interpret_slit(dr.global_map, slyce_list[i][0], x, y, slyce_list[i][1] + bm.heading_offset, 255)

	# Sorting the landmarks in terms of origin distance
	foundational_landmarks.sort(key = origin_distance, reverse = True)
	print(foundational_landmarks)

	cleaned_map = lg.clean_traverse_map(dr.global_map)
	traverse_map = cv.dilate(cleaned_map, np.ones((22, 22)), iterations = 1)

	for i in range(4):
		if (origin_distance(foundational_landmarks[i]) > 120):
			path_find(phantom_corner(foundational_landmarks[i]), ui_mode = 1)
			full_spin()
			path_find(origin, ui_mode = 1)

	display_maps()
	cv.waitKey(0)

	# Saving and loading the output
	maze = lg.clean_global_map(dr.global_map)
	cv.imshow("maze", maze)
	cv.waitKey(0)

	cv.imwrite("Automatic Map.png", maze)

if ((run_mode == 3) or (run_mode == 5)):
	slyce_list = []
	landmark_list = []

	slyce_1d = read_slit()
	dr.interpret_slit(dr.global_preview, slyce_1d, x, y, bm.get_heading(), 255)
	slyce_list.append([slyce_1d, sb.imuYaw])

	observed_landmarks = read_landmarks(slyce_1d, ui_mode = 0)
	landmark_list.append([observed_landmarks, sb.imuYaw])

	display_maps()

	# Doing a full spin to get the foundational landmarks
	for i in range(7):
		# Rotate and wait until things have settled
		bm.relative_rotate(45)
		cv.waitKey(3000)

		# Records the observation and direction of the bot
		slyce_1d = read_slit()
		slyce_list.append([slyce_1d, sb.imuYaw])

		# Records the landmarks and direction of the bot
		observed_landmarks = read_landmarks(slyce_1d)
		landmark_list.append([observed_landmarks, sb.imuYaw])

		# Displaying the movement
		dr.global_preview = dr.global_map.copy()
		render_bot(x, y, bm.get_heading(), dr.global_preview)

		foundational_landmarks = mg.map_landmarks(landmark_list, bm.heading_offset)
		display_landmarks(foundational_landmarks, color = 196)

		for i in range(len(slyce_list)):
			dr.interpret_slit(dr.global_preview, slyce_list[i][0], x, y, slyce_list[i][1] + bm.heading_offset, 255)

		display_maps()

	for i in range(len(slyce_list)):
		dr.interpret_slit(dr.global_map, slyce_list[i][0], x, y, slyce_list[i][1] + bm.heading_offset, 255)

	# Go through the manual map generation
	while True:	
		# Assigning all the keys for the user input
		print("Press Q to end map generation")
		print("Press M for relative move")
		print("Press R for relative rotate")
		print("Press S to take a scan")
		print("Press A for an A* move")
		print("Press F for a full rotate scan")
		key = input()

		# Finalise the global map and exit
		if (key == 'q'):
			cv.imwrite("generated_map.png", dr.global_map)
			break

		# Relative move by user defined input
		elif (key == 'm'):
			# Catch dumb stuff
			try: 
				dist = int(input('Distance (cm): '))
			except:
				continue

			x, y = bm.relative_move(dist, x, y)

			# Displaying the movement
			dr.global_preview = dr.global_map.copy()
			render_bot(x, y, bm.get_heading(), dr.global_preview)
			display_maps()

		# Relative rotate by user defined input
		elif (key == 'r'):
			try:
				angle = int(input('Angle (deg): '))
			except:
				continue

			bm.relative_rotate(angle)

			# Displaying the movement
			dr.global_preview = dr.global_map.copy()
			render_bot(x, y, bm.get_heading(), dr.global_preview)
			display_maps()

		# Execute an A* movement according to the user defined end point
		elif (key == 'a'):
			# Generating areas where the bot can traverse and running A*
			cleaned_map = lg.clean_traverse_map(dr.global_map)
			traverse_map = cv.dilate(cleaned_map, np.ones((22, 22)), iterations = 1)
			path, start = al.run(traverse_map, [int(x), int(y)], [330, 300], ui_mode=1)

			# Converting the basic A* vectors to something better
			path_4_ui = cv.addWeighted(traverse_map, 0.3, path, 1, 0)
			vectors = pp.convert_path_to_vectors(path, start, ui_mode=1)
			vectors_merged = pp.merge_vectors(vectors, traverse_map, path_4_ui, ui_mode=1)
			render_bot(x, y, bm.get_heading(), path_4_ui)
			cv.waitKey(100)

			# Convert to correct angles and execute movements
			for vector in vectors_merged:
				x, y = bm.execute_vector(vector, x, y)

				# Displaying the movement
				render_bot(x, y, bm.get_heading(), path_4_ui)
				cv.imshow("Path planning", path_4_ui)
				cv.waitKey(1000)

		# Spin on the spot and generate a map
		elif (key == 'f'):
			full_spin()

		# Spin but don't corner match
		elif (key == 'n'):
			slyce_list = []
			landmark_list = []

			slyce_1d = read_slit()
			slyce_list.append([slyce_1d, sb.imuYaw])

			observed_landmarks = read_landmarks(slyce_1d)
			landmark_list.append([observed_landmarks, sb.imuYaw])

			display_maps()

			# Spin around enough times
			for i in range(6):
				# Rotate and wait until things have settled
				bm.relative_rotate(45)
				cv.waitKey(3000)

				# Records the observation and direction of the bot
				slyce_1d = read_slit()
				slyce_list.append([slyce_1d, sb.imuYaw])

				# Records the landmarks and direction of the bot
				observed_landmarks = read_landmarks(slyce_1d)
				landmark_list.append([observed_landmarks, sb.imuYaw])

				display_maps()

				# Displaying the movement
				dr.global_preview = dr.global_map.copy()
				render_bot(x, y, bm.get_heading(), dr.global_preview)

				display_maps()

			print("Use WASD to correct robot position")
			print("Use ZX to correct robot heading")
			print("Press Q to solidify scan")
			print("Press R to delete last scan\n")

			while True:
				key = cv.waitKey(0) & 0xFF

				if (scan_action(key) == 1):
					rectified_landmarks = mg.map_landmarks(landmark_list, bm.heading_offset)
					display_landmarks(rectified_landmarks, x, y)

					for i in range(len(slyce_list)):
						dr.interpret_slit(dr.global_map, slyce_list[i][0], x, y, slyce_list[i][1] + bm.heading_offset, 255)

					render_bot(x, y, bm.get_heading(), dr.global_preview)
					display_landmarks(rectified_landmarks, x, y)
					display_maps()
					break
				
				render_bot(x, y, bm.get_heading(), dr.global_preview)
				display_maps()

		# Take a scan of the current view
		elif (key == 's'):
			print("Use WASD to correct robot position")
			print("Use ZX to correct robot heading")
			print("Press Q to solidify scan")
			print("Press R to delete last scan\n")

			landmark_list = []

			slyce_1d = read_slit()
			slyce_list = [[slyce_1d, sb.imuYaw]]

			observed_landmarks = read_landmarks(slyce_1d)
			landmark_list.append([observed_landmarks, sb.imuYaw])

			while True:
				key = cv.waitKey(0) & 0xFF

				if (scan_action(key) == 1):
					rectified_landmarks = mg.map_landmarks(landmark_list, bm.heading_offset)
					display_landmarks(rectified_landmarks)
					dr.interpret_slit(dr.global_map, slyce_1d, x, y, bm.get_heading(), 255)
					render_bot(x, y, bm.get_heading(), dr.global_preview)
					break
				
				render_bot(x, y, bm.get_heading(), dr.global_preview)
				display_maps()

	display_maps()
	cv.waitKey(0)

	maze = cv.imread("generated_map.png")
	maze = cv.cvtColor(maze, cv.COLOR_BGR2GRAY)

	maze = lg.clean_traverse_map(maze)
	maze = lg.clean_global_map(maze)
	cv.imshow("maze", maze)
	cv.waitKey(0)

	cv.imwrite("Manual Map.png", maze)


maze = cv.imread("generated_map.png")
maze = cv.cvtColor(maze, cv.COLOR_BGR2GRAY)

maze = lg.clean_traverse_map(maze)
maze = lg.clean_global_map(maze)


########################
#  Localise and Move   #
########################

if ((run_mode == 2) or (run_mode == 5)):
	pf.landmarks = lg.landmarks_from_global(maze, ui_mode = 1)
	
	#print(foundational_landmarks) # TEMP todo
	#for landmark in foundational_landmarks:
	#	maze[landmark[1]][landmark[0]] = 100

	cv.imshow("Landmark generation", maze)
	cv.waitKey(0)

	pf.initialise_display(maze)
	pf.initialise(cv.dilate(maze, np.ones((18, 18)), iterations = 1), ui_mode = 1)

	while True:
		# Rotating 45 degrees in a full circle and localising
		for i in range(7):
			slyce_1d = read_slit()
			observed_landmarks = read_landmarks(slyce_1d)
			
			print("What the for loop sees: ", observed_landmarks)

			# Stepping through if anything has been seen otherwise don't
			if observed_landmarks == []:
				print("No landmarks found")
			else:
				pf.update_step(observed_landmarks, ui_mode = 1)
				pf.render_particle(pf.particles[0])
				pf.resample_step(ui_mode = 1)
			
			# Actually making the bot rotate
			print("rotation start")
			bm.relative_rotate(45)
			sleep(2)
			print("rotation end")
			pf.move_step(0, 45, ui_mode = 1)

		# Saving the global map once done
		if cv.waitKey(1) & 0xFF == ord('q'):
			break
			
		# Doing the final filtering of particles
		print("final display")
		pf.update_step(observed_landmarks, ui_mode = 1)
		pf.render_particle(pf.particles[0])
		cv.imshow("Particle filter", pf.display)

		cv.waitKey(0)
		break

	print(sb.imuYaw)
	print(pf.particles[0]['rotation'] * (180 / math.pi))
	print(bm.heading_offset)
	print(pf.particles[0]['rotation'] * (180 / math.pi) - sb.imuYaw)

	pf.render_particle(pf.particles[0])

	# Get the current coordinates of the robot
	x = int(round(pf.particles[0]['position'][0]))
	y = int(round(pf.particles[0]['position'][1]))
	#bm.heading_offset = pf.particles[0]['rotation'] * (180 / math.pi) - sb.imuYaw

	# Generating areas where the bot can traverse and running A*
	traverse_map = cv.dilate(pf.obstacle_map, np.ones((22, 22)), iterations = 1)
	path, start = al.run(traverse_map, [int(x), int(y)], [330, 300], ui_mode=1)

	# Converting the basic A* vectors to something better
	path_4_ui = cv.addWeighted(traverse_map,0.3,path,1,0)
	vectors = pp.convert_path_to_vectors(path, start, ui_mode=1)
	render_bot(x, y, bm.get_heading(), path_4_ui)
	vectors_merged = pp.merge_vectors(vectors, traverse_map, path_4_ui, ui_mode=1)

	# Converting and executing the vector movements
	for vector in vectors_merged:
		x, y = bm.execute_vector(vector, x, y)

		render_bot(x, y, bm.get_heading(), path_4_ui)
		cv.imshow("Path planning", path_4_ui)
		cv.waitKey(1000)

	cv.waitKey(0)

########################
#   Show Us the Input  #
########################

if (run_mode == 4):
	while True:
		slyce_1d = read_slit()
		dr.global_preview = dr.global_map.copy()
		dr.interpret_slit(dr.global_preview, slyce_1d, x, y, bm.get_heading(), 255)
		display_maps()

		if ((cv.waitKey(100) & 0xFF) == ord('q')):
			break

print("Bye bye")
cv.waitKey(0)
sb.shutDown()

### ================================ ###
# UNUSED STUFF