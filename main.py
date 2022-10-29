### ===================================== ###
# IMPORTING EVERYTHING

from time import sleep
import math
import numpy as np
import cv2 as cv

import slamBotHD as sb
import a_star_lib as al
import path_planning_lib as pp
import depth_rectification_lib as dr
import bot_move_lib as bm
import landmark_generation_lib as lg
import particle_filtering_lib as pf

### ===================================== ###
# DEFINING FUNCTIONS

# Displays the observed and global maps
def display_maps():
	window = np.concatenate((dr.global_map, dr.global_preview), axis=1)
	cv.imshow("Map generation", window) 
	cv.waitKey(1)	

# Reads the slit of the depth sensor
def read_slit():
	depth_array = sb.imgDepth
	depth_slice = dr.convert_to_slice(depth_array)

	return depth_slice
	
# Using a given slice writes landmarks to an array
def observe_landmarks(slyce_1d):
	global view, observed_landmarks
	
	dr.local_map = np.zeros((601, 601), dtype = np.uint8)

	dr.project_slit(slyce_1d)
	view = lg.clean_local_for_lm(dr.local_map)
	
	# Extracts the landmarks from the current view
	observed_landmarks = lg.landmarks_from_slice(view, ui_mode=1)
	linked_points = dr.derectify_points(observed_landmarks)
	linked_landmarks = lg.landmark_filter(linked_points, view, ui_mode=1)

	# Clears out old observed landmarks and writes new ones
	observed_landmarks = []
	
	for point in linked_landmarks:
		observed_landmarks.append([point[0][0] - 319, point[0][1]])
		
	print("New landmarks: ", observed_landmarks)
	return

# Renders the bot with coordinates on a given display
def render_bot(x1, y1, heading, image):
	heading = heading * math.pi / 180

	x2 = x1 + int(17 * math.sin(heading))
	y2 = y1 + int(17 * math.cos(heading))

	cv.circle(image, (x1, y1), 17, 127, 1)
	cv.line(image, (x1, y1), (x2, y2), 127, 1)
		
### ===================================== ###
# INITIALISATION	
	
dr.initialise_matrices(300)
dr.initialise_tan()

sb.startUp()
sb.readCoreData()
sleep(2)

print("Battery voltage:", sb.botBatt, "V")

x = 300
y = 300
bm.heading_offset = 0

# Do you want to see a bunch of windows?
UI_mode = 1

### ===================================== ###
# ACTUAL MAIN

########################
# Map Generation Stage #
########################

# Gets the initial readings from the bot
slyce_1d = read_slit()
dr.interpret_slit(dr.global_map, slyce_1d, x, y, bm.get_heading(), 255)
display_maps()
	
# Go through the user controlled map generation
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
		# Catch dumb stuff
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
		traverse_map = cv.dilate(pf.obstacle_map, np.ones((18, 18)), iterations = 1)
		path, start = al.run(traverse_map, [int(x), int(y)], [330, 300], ui_mode=1)

		# Converting the basic A* vectors to something better
		path_4_ui = cv.addWeighted(traverse_map, 0.3, path, 1, 0)
		vectors = pp.convert_path_to_vectors(path, start, ui_mode=1)
		render_bot(x, y, bm.get_heading(), path_4_ui)
		vectors_merged = pp.merge_vectors(vectors, traverse_map, path_4_ui, ui_mode=1)

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

	# Spin on the spot and generate a map
	elif (key == 'f'):

		slyce_list = []
		full_spin = np.zeros((601, 601), dtype = np.uint8)

		slyce_1d = read_slit()
		slyce_list.append([slyce_1d, sb.imuYaw])
		dr.interpret_slit(full_spin, slyce_1d, x, y, sb.imuYaw, 255)
		display_maps()

		for i in range(7):
			# Rotate and wait until things have settled
			bm.relative_rotate(45)
			cv.waitKey(3000)

			# Records the observation and direction of the bot
			slyce_1d = read_slit()
			slyce_list.append([slyce_1d, sb.imuYaw])
			dr.interpret_slit(full_spin, slyce_1d, x, y, sb.imuYaw, 255)
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

			# display solidified map
			if (key == ord('q')):
				for i in range(8):
					dr.interpret_slit(dr.global_map, slyce_list[i][0], x, y, slyce_list[i][1]+bm.heading_offset, 255)
				display_maps()
				break

			# delete scan without solidifying
			elif (key == ord('r')):
				break

			# correct robot position/heading
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

			dr.global_preview = dr.global_map.copy()
			for i in range(8):
				dr.interpret_slit(dr.global_preview, slyce_list[i][0], x, y, slyce_list[i][1]+bm.heading_offset, 127)

			render_bot(x, y, bm.get_heading(), dr.global_preview)
			display_maps()

	# Take a scan of the current view
	elif (key == 's'):
		slyce_1d = read_slit()
		print("Use WASD to correct robot position")
		print("Use ZX to correct robot heading")
		print("Press Q to solidify scan")
		print("Press R to delete last scan\n")

		while True:
			# Display perceived map
			dr.global_preview = dr.global_map.copy()
			dr.interpret_slit(dr.global_preview, slyce_1d, x, y, bm.get_heading(), 127)
			render_bot(x, y, bm.get_heading(), dr.global_preview)
			display_maps()
			
			key = cv.waitKey(0) & 0xFF

			# Display the global and local map
			if (key == ord('q')):
				dr.interpret_slit(dr.global_map, slyce_1d, x, y, bm.get_heading(), 255)
				display_maps()
				break

			# Delete scan without solidifying
			elif (key == ord('r')):
				break

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

maze = cv.imread("generated_map.png")
maze = cv.cvtColor(maze, cv.COLOR_BGR2GRAY)

#maze = lg.clean_global_for_lm(maze)
#cv.imshow("maze", maze)
#cv.waitKey(0)

cv.imwrite("unclean.png", maze)
maze = cv.imread("cleanboi (copy).png")
pf.landmarks = lg.landmarks_from_global(maze, ui_mode=1)
cv.imshow("Landmark generation", maze)
cv.waitKey(0)
print(pf.landmarks)

########################
#  Localisation Stage  #
########################

pf.initialise_display(maze)
pf.initialise(cv.dilate(maze, np.ones((18, 18)), iterations = 1), ui_mode = 1)

while True:
	# Rotating 45 degrees in a full circle and localising
	for i in range(7):
		slyce_1d = read_slit()
		observe_landmarks(slyce_1d)
		
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
		cv.imwrite("images/generated_map.png", dr.global_map)
		break
		
	# Doing the final filtering of particles
	print("final display")
	pf.update_step(observed_landmarks, ui_mode = 1)
	pf.render_particle(pf.particles[0])
	cv.imshow("Particle filter", pf.display)

	cv.waitKey(0)
	break
	
########################
#  Pathfinding Stage   #
########################

# Get the current coordinates of the robot
x = int(round(pf.particles[0]['position'][0]))
y = int(round(pf.particles[0]['position'][1]))

# Generating areas where the bot can traverse and running A*
traverse_map = cv.dilate(pf.obstacle_map, np.ones((18, 18)), iterations = 1)
path, start = al.run(traverse_map, [int(x), int(y)], [330, 300], ui_mode=1)

# Converting the basic A* vectors to something better
if (UI_mode): path_4_ui = cv.addWeighted(traverse_map,0.3,path,1,0)
vectors = pp.convert_path_to_vectors(path, start, ui_mode=1)
render_bot(x, y, bm.get_heading(), path_4_ui)
vectors_merged = pp.merge_vectors(vectors, traverse_map, path_4_ui, ui_mode=1)

# Converting and executing the vector movements
for vector in vectors_merged:
	vector[2] -= 90

	bm.relative_rotate(bm.angle_difference(-vector[2], bm.get_heading()))
	print(bm.angle_difference(-vector[2], bm.get_heading()))
	cv.waitKey(1000)

	x, y = bm.relative_move(vector[1], x, y)
	render_bot(x, y, bm.get_heading(), path_4_ui)
	cv.imshow("Path planning", path_4_ui)
	cv.waitKey(1000)

cv.waitKey(0)

print("Bye bye")
sb.shutDown()

### ================================ ###
# UNUSED STUFF

'''	
# Loads the global map and finds all the corners
def load_global_map():
	global maze

	maze = cv.imread("cleaned.png")
	pf.landmarks = lg.landmarks_from_global(cv.cvtColor(maze, cv.COLOR_BGR2GRAY), ui_mode = 0)
'''

'''	
	for i in range(5):
		pf.update_step(observed_landmarks, ui_mode = 1)
		pf.resample_step(ui_mode = 1)
		
	print("move start")
	bm.relative_move(50)
	sleep(2)
	print("move done")	
	pf.move_step(50, 0, ui_mode = 1)
	pf.particle_filter(observed_landmarks, ui_mode = 1)
	
	for i in range(5):
		pf.update_step(observed_landmarks, ui_mode = 1)
		pf.resample_step(ui_mode = 1)
'''

'''
path, start = a_star.run(maze, [64, 18], [152, 58], ui_mode=1)

if (UI_mode): path_4_ui = cv.addWeighted(maze,0.3,path,1,0)

vectors = pp.convert_path_to_vectors(path, start, 1)

vectors_merged = pp.merge_vectors(vectors, maze, path_4_ui, 1)

'''
		
'''	
	bm.relative_rotate(90)
	sleep(2)
	pf.move_step(0, math.pi/2, ui_mode = 0)
	pf.update_step(observed_landmarks, ui_mode = 0)
	pf.resample_step(ui_mode = 0)
	
	for i in range(5):
		pf.move_step(0, 0, ui_mode = 0)
		pf.update_step(observed_landmarks, ui_mode = 0)
		pf.resample_step(ui_mode = 0)
'''
	

'''
while True:
	depth_array = sb.imgDepth
	ang_deg = sb.imuYaw
	while(ang_deg==None):
		sleep(1)
		print("null")
	ang_rad = ang_deg * 3.14159265358793 / 180

	slyce = depth_array[236:244][:]
	slyce = dr.depth_to_greyscale(slyce)
	slyce_1d = dr.avg_slice(slyce)
	
	
	dr.local_map = np.zeros((601, 601), dtype = np.uint8)
	dr.project_slit(slyce_1d)
	view = lg.clean_image_for_lm(dr.local_map)
	
	uncorrected_view = dr.uncorrected_slit(slyce_1d)
	uncorrected_view = lg.clean_image_for_lm(uncorrected_view)
	
	cv.imshow("local map", view) 
	
	observed_landmarks = lg.landmarks_from_slice(view, ui_mode=1)
	linked_points = dr.derectify_points(observed_landmarks)
	linked_landmarks = lg.landmark_filter(linked_points)
	
	for point in linked_landmarks:
		print(point[0])
	
	for landmark in linked_landmarks:
		cv.circle(uncorrected_view,(landmark[1][0],landmark[1][1]), 5, 128, -1)

	cv.imshow("local unco map", uncorrected_view) 
	
	if cv.waitKey(1) & 0xFF == ord('q'):
		cv.imwrite("generated_map.png", dr.global_map)
		break
'''

'''
dr.local_map = cv.blur(dr.local_map, (5, 5))
ret, thresh = cv.threshold(dr.local_map, 50, 255, cv.THRESH_BINARY)
dr.local_map = thresh
dr.local_map = cv.erode(dr.local_map, np.ones((2, 2)), iterations = 1)
dr.local_map = cv.dilate(dr.local_map, np.ones((3, 3)),iterations = 1)
'''

'''
map = cv.imread("map.png")
map = cv.cvtColor(map, cv.COLOR_BGR2GRAY)
#start = [0,0]
path, start = a_star.run(map, [64, 18], [152, 58], ui_mode=1)


if (UI_mode): path_4_ui = cv.addWeighted(map,0.3,path,1,0)


vectors = path_planner.convert_path_to_vectors(path, start, 0)

path_planner.merge_vectors(vectors, map, path_4_ui, UI_mode) #obstacle check potentially not working
'''