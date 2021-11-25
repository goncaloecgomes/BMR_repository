import cv2 as cv
import numpy as np
import pyvisgraph_master.pyvisgraph as vg
import corner as cn
import Polygon as poly

image = cv.imread("data/poligonos.png") 

vg_poly_list = poly.get_polygons_from_image(image)

start_point = vg.Point(10,10) #sink

end_point = vg.Point(200,200) #goal

g = vg.VisGraph()
g.build(vg_poly_list)
shortest = g.shortest_path(start_point, end_point)

print("\n\n")
print("Shortest path:", shortest)
print("\n\n")

cv.imshow('dst', image)
cv.waitKey(0)
cv.destroyAllWindows()