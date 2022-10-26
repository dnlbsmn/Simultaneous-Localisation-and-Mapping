### ===================================== ###
# IMPORTING EVERYTHING

import a_star
import path_planner as pp
import cv2 as cv
import slamBotHD as sb
import depth_rect_lib as dr
import numpy as np
import bot_move_lib as bm
import landmark_generator as lg
import particle_lib as pl
from time import sleep
import math

UI_mode = 1

### ===================================== ###
# DEFINING FUNCTIONS

def initialise():
	dr.initialise_matrices(300)
	dr.initialise_tan()

	sb.startUp()
	sb.readCoreData()
	sleep(2)

	print("Battery voltage:", sb.botBatt, "V")
	
def display_local_map():
	global view

	cv.imshow("local map", view) 
	#cv.imwrite("sample_corner_1", view)
	cv.waitKey(1)	

def display_global_map():
	cv.imshow("global map", dr.global_map) 
	#cv.imwrite("sample_corner_1", view)
	#dr.global_map  = lg.clean_image_for_lm(dr.global_map)
	cv.waitKey(1)	

def init_slit():
	depth_array = sb.imgDepth
	depth_slice = dr.convert_to_slice(depth_array)

	return depth_slice
	
def observe_landmarks(slyce_1d):
	global view, observed_landmarks
	
	dr.local_map = np.zeros((601, 601), dtype = np.uint8)

	dr.project_slit(slyce_1d)
	view = lg.clean_local_for_lm(dr.local_map)
	bare_view = view

	#cv.imshow('Harris Corners', view)
	#cv.waitKey(0)
	
	observed_landmarks = lg.landmarks_from_slice(view, ui_mode=1)
	linked_points = dr.derectify_points(observed_landmarks)
	linked_landmarks = lg.landmark_filter(linked_points, bare_view, ui_mode=1)
	

	observed_landmarks = []
	
	for point in linked_landmarks:
		observed_landmarks.append([point[0][0] - 319, point[0][1]])
		
	print("New landmarks: ", observed_landmarks)
	return 
		
def load_global_map():
	global maze

	maze = cv.imread("cleaned.png")
	pl.landmarks = lg.landmarks_from_global(cv.cvtColor(maze, cv.COLOR_BGR2GRAY), ui_mode = 0)

def render_bot(x1, y1, heading, image):
	heading = heading * math.pi / 180

	x2 = x1 + int(17 * math.sin(heading))
	y2 = y1 + int(17 * math.cos(heading))

	cv.circle(image, (x1, y1), 17, 127, 1)
	cv.line(image, (x1, y1), (x2, y2), 127, 1)
		
### ===================================== ###
# INITIALISATION	
	
initialise()
load_global_map()



x = 300
y = 300


### ===================================== ###
# TESTBENCH

########################
# Map generation stage
########################

#slyce_1d = init_slit()
#dr.interpret_slit(dr.global_map, slyce_1d, x, y, bm.get_heading(), 255)
#display_global_map()

# Spin on the spot and generate a map
for i in range(0):
	
	bm.relative_rotate(45)
	cv.waitKey(3000)

	slyce_1d = init_slit()
	dr.interpret_slit(dr.global_map, slyce_1d, x, y, bm.get_heading(), 255)
	display_global_map()

	dr.global_preview = dr.global_map.copy()
	render_bot(x, y, bm.get_heading(), dr.global_preview)
	cv.imshow("Preview", dr.global_preview)
	cv.waitKey(10)
	
# Go through the user controlled map generation
while False:	
	print("Press Q to end map generation")
	print("Press M for relative move")
	print("Press R for relative rotate")
	print("Press S to take a scan")
	key = cv.waitKey(0) & 0xFF

	# finalise map
	if (key == ord('q')):
		cv.imwrite("generated_map.png", dr.global_map)
		break

	# relative move by user defined input
	elif (key == ord('m')):
		try: 
			dist = int(input('Distance (cm): '))
		except:
			continue
		x, y = bm.relative_move(dist, x, y)
		dr.global_preview = dr.global_map.copy()
		render_bot(x, y, bm.get_heading(), dr.global_preview)
		cv.imshow("Preview", dr.global_preview)

	# relative rotate by user defined input
	elif (key == ord('r')):
		try:
			angle = int(input('Angle (deg): '))
		except:
			continue
		bm.relative_rotate(angle)
		dr.global_preview = dr.global_map.copy()
		render_bot(x, y, bm.get_heading(), dr.global_preview)
		cv.imshow("Preview", dr.global_preview)

	# take a scan 
	elif (key == ord('s')):
		slyce_1d = init_slit()
		print("Use WASD to correct robot position")
		print("Use ZX to correct robot heading")
		print("Press Q to solidify scan")
		print("Press R to delete last scan\n")

		while True:
			# display perceived map
			dr.global_preview = dr.global_map.copy()
			dr.interpret_slit(dr.global_preview, slyce_1d, x, y, bm.get_heading(), 127)
			render_bot(x, y, bm.get_heading(), dr.global_preview)
			cv.imshow("Preview", dr.global_preview)
			
			key = cv.waitKey(0) & 0xFF

			# display solidified map
			if (key == ord('q')):
				dr.interpret_slit(dr.global_map, slyce_1d, x, y, bm.get_heading(), 255)
				display_global_map()
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
		
########################
# Localisation stage
########################

pl.initialise_display(maze)
pl.initialise(maze, ui_mode = 1)

while True:
	#observe_landmarks()
	#display_local_map()

	for i in range(7):
		slyce_1d = init_slit()
		observe_landmarks(slyce_1d)

		#head = sb.imuYaw
		#dr.interpret_slit(slyce_1d, x, y, head)
		#display_global_map()
		display_local_map()
		
		print("What the for loop sees: ", observed_landmarks)

		if observed_landmarks == []:
			print("No landmarks found")
		else:
			for i in range(1):
				pl.update_step(observed_landmarks, ui_mode = 1)
				pl.render_particle(pl.particles[0])
				pl.resample_step(ui_mode = 1)
		
		print("rotation start")
		bm.relative_rotate(45)
		sleep(2)
		print("rotation end")
		pl.move_step(0, 45, ui_mode = 1)
		print("display done")


	if cv.waitKey(1) & 0xFF == ord('q'):
		cv.imwrite("generated_map.png", dr.global_map)
		break
		
	print("final display")
	pl.update_step(observed_landmarks, ui_mode = 1)
	pl.render_particle(pl.particles[0])
	cv.imshow("display", pl.display)

	cv.waitKey(0)
	break
	
sb.shutDown()

########################
# A STAR STAGE
########################

### ================================ ###
# UNUSED STUFF

'''	
	for i in range(5):
		pl.update_step(observed_landmarks, ui_mode = 1)
		pl.resample_step(ui_mode = 1)
		
	print("move start")
	bm.relative_move(50)
	sleep(2)
	print("move done")	
	pl.move_step(50, 0, ui_mode = 1)
	pl.particle_filter(observed_landmarks, ui_mode = 1)
	
	for i in range(5):
		pl.update_step(observed_landmarks, ui_mode = 1)
		pl.resample_step(ui_mode = 1)
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
	pl.move_step(0, math.pi/2, ui_mode = 0)
	pl.update_step(observed_landmarks, ui_mode = 0)
	pl.resample_step(ui_mode = 0)
	
	for i in range(5):
		pl.move_step(0, 0, ui_mode = 0)
		pl.update_step(observed_landmarks, ui_mode = 0)
		pl.resample_step(ui_mode = 0)
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

