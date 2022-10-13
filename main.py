import a_star
import path_planner
import cv2 as cv
import slamBotHD as sb
import depth_rect_lib as dr
import numpy as np
import bot_move_lib as BML
from time import sleep

UI_mode = 1

def display_array(array, window = "default"):
	try:
		cv.namedWindow(window, cv.WINDOW_NORMAL)
		cv.resizeWindow(window, (640, 480))
		cv.imshow(window, array)
		cv.waitKey(500)
		#cv.destroyAllWindows()
	except:
		cv.waitKey(0)
		return

print(sb.botBatt)

map = cv.imread("map.png")
map = cv.cvtColor(map, cv.COLOR_BGR2GRAY)
#start = [0,0]
path, start = a_star.run(map, [64, 18], [152, 58], ui_mode=1)


if (UI_mode): path_4_ui = cv.addWeighted(map,0.3,path,1,0)


vectors = path_planner.convert_path_to_vectors(path, start, 0)

path_planner.merge_vectors(vectors, map, path_4_ui, UI_mode) #obstacle check potentially not working




'''
print("loop")
cv.namedWindow("display", cv.WINDOW_NORMAL)
cv.resizeWindow("display", 1280, 960)

dr.initialise_matrices(300)
dr.initialise_tan()

sb.startUp()
sb.readCoreData()
sleep(1)

while True:
	
	depth_array = sb.imgDepth
	ang_deg = sb.imuYaw
	ang_rad = ang_deg*3.14159265358793/180

	slyce = depth_array[236:244][:]
	slyce = dr.depth_to_greyscale(slyce)
	slyce_1d = dr.avg_slice(slyce)
	
	cv.imshow("display", dr.global_map) 

	dr.interpret_slit(slyce_1d, 301, 301, -ang_rad)
	
	if cv.waitKey(1) & 0xFF == ord('q'):
		cv.imwrite("generated_map.png", dr.global_map)
		break

sb.shutDown()'''

