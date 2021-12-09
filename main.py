import cv2 as cv
import numpy as np
import pyvisgraph_master.pyvisgraph as vg
import corner as cn
import Polygon as poly
import video as vd
import Kalman as kal

import sys
sys.setrecursionlimit(5000)

#----------------------------------------------------------------------------------------------------------------
#    FIND SHORTEST PATH
#----------------------------------------------------------------------------------------------------------------
# Capturing the map to find the shortest path
<<<<<<< HEAD
#cap = cv.VideoCapture(2) 
#isTrue, frame = cap.read()
=======
cap = cv.VideoCapture(1 + cv.CAP_DSHOW)
isTrue, frame = cap.read()
>>>>>>> 2d45ee34a1c35b4dbc0c5291668ec3625ca832ce

# img = vd.rescaleFrame(frame, scale=0.5)
# source = cv.imread("source.png")
# sink = cv.imread("sink.png")
#img_to_find = cv.imread("poligonos_test.png")
# img = cv.imread("data/poligonos.png")


# cap = cv.VideoCapture(2)
# count = 0 
# while count<50:
#     ret, img = cap.read()
#     count += 1

Thymio_size = 30

img = cv.imread("data/Real_Env1.jpeg")
#img = img[100:399,55:575]


print("Image Len:", img.shape)
maxX,maxY,_ = img.shape

# ----------------- FIND SOURCE ------------------------

# source, boxSource =

# -----------------------------------------------------

virtual_image = vd.draw_objects_in_Virtual_Env(img)

#------------------ ERASE QRCODE FROM VIRTUAL ENV

#cv.drawContours(virtual_image, [boxSource], 0, (255,255,255),-1)

#---------------------------------------------------------------

sink, poly_list = poly.get_polygons_from_image(virtual_image,False,True)


augmented_poly_list = poly.augment_polygons(poly_list,maxX,maxY,Thymio_size)


virtual_image = poly.draw_polygons_in_image(virtual_image,augmented_poly_list)

thickness = 3

virtual_image = cv.rectangle(virtual_image,(thickness,thickness),(maxY-thickness,maxX-thickness),(255,255,255),thickness*2)

new_poly_list = poly.get_polygons_from_image(virtual_image,False,False)

new_poly_list = poly.poly_out_of_bound(new_poly_list,maxX,maxY, Thymio_size)

vg_poly_list = poly.transform_poly_in_vg_poly(new_poly_list)



g = vg.VisGraph()
g.build(vg_poly_list)

#(sourceX, sourceY) = vd.findObject(template_img=cv.imread("data/source.png"), img_to_find= sink_goal)
start_point = vg.Point(source[0], source[1]) #start --> We need to change this to the initial Position of the Thymio [TODO]

#(sinkX, sinkY) = vd.findObject(template_img=cv.imread("data/sink.png"), img_to_find= sink_goal)
end_point = vg.Point(sink[0], sink[1]) #goal --> We need to change this to the position of wtv we define is our goal [TODO]

shortest = g.shortest_path(start_point, end_point)

print("\n\n")
print("Shortest path:", shortest)
print("\n\n")

for _poly in poly_list:
    for point in _poly:
        cv.circle(img, (int(point[1]), int(point[0])), radius=5, color=(255, 0, 255), thickness=-1)

for point in shortest:
    sink_goal = cv.circle(img, (int(point.y), int(point.x)), radius=5, color=(0, 255, 0), thickness=-1)

cv.imshow("VE", virtual_image)
cv.imshow('frame', img)
cv.waitKey(0)

#cap.release()
cv.destroyAllWindows()

# [TODO] send shortest path to the Thymio

#----------------------------------------------------------------------------------------------------------------
#    TRACKING THE THYMIO
#----------------------------------------------------------------------------------------------------------------

while (True):
    ret, frame = cap.read()

    # frame_resize = rescale(frame, scale=0.5)

    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    cv.imshow('frame', gray)
    

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cv.waitKey(0)
cap.release()
cv.destroyAllWindows()

# cv.imshow('dst', frame)
# cv.waitKey(0)
# cv.destroyAllWindows()

#----------------------------------------------------------------------------------------------------------------
#   KALMAN FILTER PART
#----------------------------------------------------------------------------------------------------------------

#  initial conditions
X_est = np.array([[4000],   # x position
                  [280],    # x velocity
                  [4000],    # y position
                  [280]])  # y velocity

P_est = kal.covariance(20, 5, 20, 5)  # state covariance matrix

Q = kal.covariance(20, 5, 20, 5)  # process noise covariance matrix

R = kal.covariance(25, 6, 25, 6)  # measurement covariance matrix

#  some test observations
x_obs = np.array([4000, 4260, 4550, 4860, 5110])
xdot_obs = np.array([280, 282, 285, 286, 290])
y_obs = np.array([4000, 4260, 4550, 4860, 5110])
ydot_obs = np.array([280, 282, 285, 286, 290])

# example loop taking the test observations and the previous state to update the current state
for i in range(len(x_obs)):
    X_est, P_est = kal.kalman_filter(X_est, P_est, Q, R, x_obs[i], xdot_obs[i], y_obs[i], ydot_obs[i])
    print(X_est)

