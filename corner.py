import cv2
import numpy as np
"""https://stackoverflow.com/questions/55039229/how-to-find-corners-of-an-object-using-opencv"""

def find_centroids(dst):
    ret, dst = cv2.threshold(dst, 0.01 * dst.max(), 255, 0)
    dst = np.uint8(dst)

    # find centroids
    ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)
    # define the criteria to stop and refine the corners
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 
                0.001)
    corners = cv2.cornerSubPix(gray,np.float32(centroids[1:]),(5,5), 
          (-1,-1),criteria)
    return corners

image = cv2.imread("poligons3.png")
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

gray = np.float32(gray)

dst = cv2.cornerHarris(gray, 3, 3, 0.04)

dst = cv2.dilate(dst, None)

# Threshold for an optimal value, it may vary depending on the image.
# image[dst > 0.01*dst.max()] = [0, 0, 255]

# Get coordinates
corners = find_centroids(dst)
corners_list = []
# To draw the corners
for corner in corners:
    image[int(corner[1]), int(corner[0])] = [0, 0, 255]
    corners_list.append([int(round(corner[1])),int(round(corner[0]))])
int_corners = np.asarray(corners, dtype = int)
print (int_corners)
print ("Pixels for corner 1 is: ", int_corners[0])
print ("Pixels for corner 2 is: ", int_corners[1])
print ("Pixels for corner 3 is: ", int_corners[2])

cv2.imshow('dst', dst)
cv2.imwrite('corners.jpg', image)
cv2.waitKey(0)
cv2.destroyAllWindows()





