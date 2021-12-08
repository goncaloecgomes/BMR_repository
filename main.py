import cv2 as cv
import numpy as np
import pyvisgraph_master.pyvisgraph as vg
import corner as cn
import Polygon as poly
import video as vd
import Kalman as kal

#----------------------------------------------------------------------------------------------------------------
#    FIND SHORTEST PATH
#----------------------------------------------------------------------------------------------------------------
# Capturing the map to find the shortest path
cap = cv.VideoCapture(1 + cv.CAP_DSHOW)
isTrue, frame = cap.read()

# img = vd.rescaleFrame(frame, scale=0.5)
source = cv.imread("source.png")
sink = cv.imread("sink.png")
#img_to_find = cv.imread("poligonos_test.png")
img = cv.imread("data/poligonos.png")

vg_poly_list = poly.get_polygons_from_image(img)

g = vg.VisGraph()
g.build(vg_poly_list)

(sourceX, sourceY) = vd.findObject(template_img=source, img_to_find= cv.imread("poligonos_test.png"))
start_point = vg.Point(sourceY, sourceX) #start --> We need to change this to the initial Position of the Thymio [TODO]

(sinkX, sinkY) = vd.findObject(template_img=sink, img_to_find= cv.imread("poligonos_test.png"))
end_point = vg.Point(sinkY, sinkX) #goal --> We need to change this to the position of wtv we define is our goal [TODO]

shortest = g.shortest_path(start_point, end_point)

print("\n\n")
print("Shortest path:", shortest)
print("\n\n")

for point in shortest:
    img = cv.circle(img, (int(point.y), int(point.x)), radius=5, color=(0, 0, 255), thickness=-1)

cv.imshow("Image", img)
cv.waitKey(0)


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

