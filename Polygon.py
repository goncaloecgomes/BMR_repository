import cv2 as cv
import numpy as np
import pyvisgraph_master.pyvisgraph as vg
import corner as cn
import itertools
import shapely
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon


def get_poly_centroid(Polygon, maxX, maxY):
    """return centroid of Polygon with maxX and maxY the limits of the image"""
    _x_list = []
    _y_list = []
    for vertex in Polygon:
        if (vertex[0] < maxX) & (vertex[1]<maxY):
            _x_list.append(vertex[0])
            _y_list.append(vertex[1])
    _len = len(_x_list)
    _x = int(sum(_x_list) / _len)
    _y = int(sum(_y_list) / _len)
    return[_x, _y]


def get_edges(image):
    """return pixelmatrix with edges and list of edges"""
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    edges = cv.Canny(gray,100,200)

    #edges = cv.dilate(edges, None)

    edge_list = []
    for coord in np.argwhere(edges >= 250):
        edge_list.append(list(coord))

    return edges, edge_list

def build_poly(point, edges ,edge_list, corners_list, poly, maxX, maxY, is_point_corner):
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

    if is_point_corner & (edges[point[0], point[1]] > 250):
        print("\nCORNER:",point)
        poly.append(point)

    edges[point[0], point[1]] = 0


    for path_point in path:
        next = list(np.array(point) + path_point)
        if (next[0] < maxX) & (next[1] < maxY):
            if(edges[next[0], next[1]] > 250):
                if next in corners_list:
                    build_poly(next,edges, edge_list, corners_list, poly,maxX,maxY,True)
                # else:
                #     build_poly(next,edges, edge_list, corners_list, poly,maxX,maxY, False)

    for path_point in path:
        next = list(np.array(point) + path_point)
        if (next[0] < maxX) & (next[1] < maxY):
            if(edges[next[0], next[1]] > 250):
                if next not in corners_list:
                    build_poly(next,edges, edge_list, corners_list, poly,maxX,maxY, False)
        else:
            OF_point = [next[0],next[1]]
            if next[0]>=maxX:
                OF_point[0] = maxX*2
            if next[1]>=maxY:
                OF_point[1] = maxY*2
            poly.append(OF_point)
    return poly


def get_polygons_from_image(image, show = False, need_source_sink =True):
    """return a list of polygons represented as corners"""
    dst,_,corners_list = cn.coord_corners(image) 
    maxX, maxY, _ = image.shape
    edges,edge_list = get_edges(image)
    print("corner_list:",corners_list,"\n")
    print("corner_list_len:", len(corners_list),"\n")
    print("maxX:\n",maxX)
    print("maxY:\n", maxY)
    print("edges:\n", edges.shape)

    poly = []
    poly_list=[]


    for corner in corners_list:
        edges[corner[0], corner[1]] = 255


 
    for corner in corners_list:
        is_point_corner = True
        poly_list.append(build_poly(corner,edges, edge_list, corners_list, poly,maxX,maxY,is_point_corner))
        poly= []

    poly_list = [x for x in poly_list if len(x) != 0]

    
    if show == True:
        cv.imshow('frame', dst)
        #Destroy all the windows
        cv.waitKey(0)
        cv.destroyAllWindows()

 
    for element in poly_list:
        print(element)
        print("\n\n")

    remove_list =[]
    for poly in poly_list:
        centroid = get_poly_centroid(poly,maxX,maxY)
        print(centroid)
        

        bgr_values = image[int(centroid[0]),int(centroid[1])]
        print(bgr_values)

        if(np.linalg.norm(np.array(bgr_values)-np.array([0,0,0])) >= 50):
            if(np.linalg.norm(np.array(bgr_values)-np.array([255,0,0])) <= 50):
                print("sink")
                sink = centroid
            remove_list.append(poly)

    print("Poligons to be removed:", remove_list)
    for poly in remove_list:
        poly_list.remove(poly)

    if need_source_sink:
        return sink, poly_list
    else:
        return poly_list

def augment_polygons(poly_list,maxX,maxY, Thymio_size):
    """return a list of polygons augmented based on the size of thymio"""
    augmented_poly_list = []

    
    for poly in poly_list:
        
        scaled_augmented_poly = []
        scaled_augmented_points = []
        centroid = get_poly_centroid(poly,maxX,maxY)
        distance_from_centroid = np.linalg.norm(np.array(centroid)-np.array(poly[0]))
        scale = 1 + (Thymio_size*1.2)/distance_from_centroid

        augmented_poly = np.array(poly) - np.array(centroid)
        augmented_poly = augmented_poly * scale
        augmented_poly = augmented_poly + np.array(centroid)
        augmented_poly = augmented_poly.tolist()

        for augmented_point in augmented_poly:
            if (augmented_point[0]<maxX) & (augmented_point[1]< maxY): 
                x = int(augmented_point[0])
                y = int(augmented_point[1])
                scaled_augmented_poly.append([x,y])
            else:
                x = int(augmented_point[0])
                y = int(augmented_point[1])
                scaled_augmented_poly.append([x,y])
        augmented_poly_list.append(scaled_augmented_poly)

    return augmented_poly_list

def remove_duplicates(poly):
    """remove duplicate points in polygon"""
    poly = sorted(poly)
    poly = list(k for k, _ in itertools.groupby(poly))
    return poly


def transform_poly_in_vg_poly(poly_list):
    """transform a list of polygons into a a list of polygons in vg_format"""
    vg_poly = []
    vg_poly_list= []

    for poly in poly_list:
        for coord in poly:
            vg_poly.append(vg.Point(coord[0], coord[1]))
        vg_poly_list.append(vg_poly)
        vg_poly = []
    return vg_poly_list


def draw_polygons_in_image(img,polys):
    """draw polygons in image"""
    for poly in polys:
        draw_poly = [ [x[1],x[0]] for x in poly ]
        print("\nPolygon to draw:",draw_poly,"\n")
        pts = np.array(draw_poly)
        img = cv.fillPoly(img, np.int32([pts]), (0,0,0))

    return img


def poly_out_of_bound(poly_list,maxX,maxY, thymio_size):
    """return a list of polygons with out of bound points"""
    for poly in poly_list:
        for corner in poly:
            if corner[0] < thymio_size:
                corner[0] = -2*maxX
            if corner[0] > (maxX-thymio_size):
                corner[0] = 2*maxX
            if corner[1] < thymio_size:
                corner[1] = -2*maxY
            if corner[1] > (maxY-thymio_size):
                corner[1] = 2*maxY

    return poly_list