import cv2 as cv
import numpy as np
import pyvisgraph_master.pyvisgraph as vg
"""https://stackoverflow.com/questions/55039229/how-to-find-corners-of-an-object-using-opencv"""



def find_centroids(dst, gray):
    """return the central corner based on cornerHarris output"""
    ret, dst = cv.threshold(dst, 0.01 * dst.max(), 255, 0)
    dst = np.uint8(dst)

    # find centroids
    ret, labels, stats, centroids = cv.connectedComponentsWithStats(dst)
    # define the criteria to stop and refine the corners
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 
                0.001)
    corners = cv.cornerSubPix(gray,np.float32(centroids[1:]),(5,5), 
          (-1,-1),criteria)
    return corners


def coord_corners (image):
    """return pixelmatrix of corners, coordenates of the coorner's shapes in the image in format of np.array and list"""

    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

    gray = np.float32(gray)

    gray = cv.GaussianBlur(gray,(9,9),0)

    dst = cv.cornerHarris(gray, 3, 3, 0.04)

    dst = cv.dilate(dst, None)

    corners = find_centroids(dst, gray)
    corners_list = []
    # To draw the corners   
    for corner in corners:
        image[int(corner[1]), int(corner[0])] = [0, 0, 255]
        corners_list.append([int(round(corner[1])),int(round(corner[0]))])
    int_corners = np.asarray(corners, dtype = int)
    return dst,int_corners, corners_list


def get_edges(image):
    """return pixelmatrix with edges and list of edges"""
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    gray_blur = cv.GaussianBlur(gray, (3,3), 0) 
    edges = cv.Canny(gray_blur,100,200)

    edge_list = []
    for coord in np.argwhere(edges >= 250):
        edge_list.append(list(coord))

    return edges, edge_list


def build_poly( point, edges ,edge_list, corners_list, poly, is_point_corner):
    """return the corners of polygon"""
   
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
            build_poly(next,edges, edge_list, corners_list, poly, False)


    return poly


def get_polygons_from_image(image):
    """return a list of polygons represented as corners"""
    _,_,corners_list = coord_corners(image) 

    edges,edge_list = get_edges(image)

    poly = []
    poly_list=[]

    for corner in corners_list:
        is_point_corner = True
        poly_list.append(build_poly(corner,edges, edge_list, corners_list, poly, is_point_corner))
        poly= []

    for element in poly_list:
        print(element)
        print("\n\n")

    vg_poly = []
    vg_poly_list= []

    for poly in poly_list:
        for coord in poly:
            vg_poly.append(vg.Point(coord[0], coord[1]))
        vg_poly_list.append(vg_poly)
        vg_poly = []

    return vg_poly_list







