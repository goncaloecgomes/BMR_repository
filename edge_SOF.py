import cv2 as cv
import numpy as np


image = cv.imread("poligonos2.png")
gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
gray_blur = cv.GaussianBlur(gray, (3,3), 0) 
edges = cv.Canny(gray_blur,100,200)



def build_poly(dst, point, edge_list, corners_list, poly, is_point_corner):

   
    up = np.array([-1,0])
    down = np.array([1,0])
    left = np.array([0,-1])
    right = np.array([0,1])
    up_right = up + right
    up_left = up + left
    down_right = down +right
    down_left = down + left

    path = [up, up_right, right, down_right, down, down_left, left, up_left]

    if(is_point_corner):
        poly.append(point)
        corners_list.remove(point)

    edges[point[0], point[1]] = 0

    for path_point in path:
        next = list(np.array(point) + path_point)

        if next in corners_list:
            poly.append(next)
            corners_list.remove(next)

    
    for path_point in path:
        next = list(np.array(point) + path_point)
        if(edges[next[0], next[1]] > 250):
            build_poly(dst, next, edge_list, corners_list, poly, False)


    return poly





def find_centroids(dst):
    ret, dst = cv.threshold(dst, 0.001 * dst.max(), 255, 0)
    dst = np.uint8(dst)

    # find centroids
    ret, labels, stats, centroids = cv.connectedComponentsWithStats(dst)
    # define the criteria to stop and refine the corners
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 
                0.001)
    corners = cv.cornerSubPix(gray,np.float32(centroids[1:]),(5,5), 
          (-1,-1),criteria)
    return corners


gray = np.float32(gray)

dst = cv.cornerHarris(gray, 3, 3, 0.04)

dst = cv.dilate(dst, None)

# Threshold for an optimal value, it may vary depending on the image.
# image[dst > 0.01*dst.max()] = [0, 0, 255]

# Get coordinates
corners = find_centroids(dst)

corners_list = []
# To draw the corners
for corner in corners:
    #image[int(corner[1]), int(corner[0])] = [0, 0, 255]
    corners_list.append([int(round(corner[1])),int(round(corner[0]))])
int_corners = np.asarray(corners, dtype = int)

#print (corners_list)

edge_list = []
for coord in np.argwhere(edges >= 250):
    edge_list.append(list(coord))

#print(edge_list[0])

poly = []
poly_list=[]
for corner in corners_list:
    is_point_corner = True
    poly_list.append(build_poly(dst, corner, edge_list, corners_list, poly, is_point_corner))
    poly= []

for element in poly_list:
    print(element)
    print("\n\n")







cv.imshow('dst', image)
cv.waitKey(0)
cv.destroyAllWindows()