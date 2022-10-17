import a_star
import path_planner
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

def initialise():
	dr.initialise_matrices(300)
	dr.initialise_tan()

	sb.startUp()
	sb.readCoreData()
	sleep(2)

	print(sb.botBatt)

initialise()

maze = cv.imread("perfect_map.png")
maze = cv.cvtColor(maze, cv.COLOR_BGR2GRAY)
	
cv.imshow('Harris Corners', maze)
cv.waitKey(0)
	
pl.landmarks = lg.extract_landmarks(maze, ui_mode = 1)

maze = cv.imread("perfect_map.png")
maze = cv.cvtColor(maze, cv.COLOR_BGR2GRAY)

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

	cv.imshow("local map", view) 
	
	observed_landmarks = lg.landmarks_from_slice(view, ui_mode=1)
	linked_points = dr.derectify_points(observed_landmarks)
	linked_landmarks = lg.landmark_filter(linked_points)
	
	observed_landmarks = []
	
	for point in linked_landmarks:
		observed_landmarks.append([point[0][0] - 319, point[0][1]])

	pl.initialise_display(maze)
	pl.initialise(ui_mode = 1)
	
	for i in range(4):
	
		pl.move_step(0, 0, ui_mode = 1)
		pl.particle_filter(observed_landmarks, ui_mode = 1)
	
	bm.relative_rotate(90)
	sleep(2)

	pl.move_step(0, math.pi/2, ui_mode = 1)
	pl.particle_filter(observed_landmarks, ui_mode = 1)
	
	for i in range(100):
		pl.move_step(0, 0, ui_mode = 1)
		pl.particle_filter(observed_landmarks, ui_mode = 1)
	
	if cv.waitKey(1) & 0xFF == ord('q'):
		cv.imwrite("generated_map.png", dr.global_map)
		break

sb.shutDown()

### ================================ ###
# UNUSED STUFF

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

