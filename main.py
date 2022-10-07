import a_star
import path_planner
import cv2 as cv

map = cv.imread("map.png")
map = cv.cvtColor(map, cv.COLOR_BGR2GRAY)
#start = [0,0]
path, start = a_star.run(map, [0, 0], [207, 157], ui_mode=0)

vectors = path_planner.convert_path_to_vectors(path, start, 0)
print(vectors)