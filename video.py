import numpy as np
import Polygon as poly
import imutils
import cv2 as cv
import random as rng
import pyzbar
from pyzbar import pyzbar
import qrcode
rng.seed(12345)

# Reshaping
def rescaleFrame(frame, scale):
    """This function resizes the the input image (frame) by a certain scale"""
    width = int(frame.shape[1]*scale)
    height = int(frame.shape[0]*scale)

    dimensions = (width, height)

    return cv.resize(frame, dimensions, interpolation = cv.INTER_AREA)


def findObject(template_img, img_to_find):
    """This function finds a specific object (template image) in an image (img_to_find)"""
    # Source: https://www.pyimagesearch.com/2015/01/26/multi-scale-template-matching-using-python-opencv/

    # load the image image, convert it to grayscale, and detect edges
    template = rescaleFrame(template_img, scale=0.8)
    template = cv.cvtColor(template_img, cv.COLOR_BGR2GRAY)
    #template = cv.Canny(template, 50, 200)
    (tH, tW) = template.shape[:2]
    #cv.imshow("Template", template)


    gray = cv.cvtColor(img_to_find, cv.COLOR_BGR2GRAY)
    gray = rescaleFrame(gray, scale=0.8)

    # cv.imshow("Image2", template)
    # cv.waitKey(0)
    #gray = img_to_find
    found = None

    # loop over the scales of the image
    for scale in np.linspace(0.2, 1.0, 50)[::-1]:
        # resize the image according to the scale, and keep track
        # of the ratio of the resizing
        resized = imutils.resize(gray, width = int(gray.shape[1] * scale))
        r = gray.shape[1] / float(resized.shape[1])
        # if the resized image is smaller than the template, then break
        # from the loop
        if resized.shape[0] < tH or resized.shape[1] < tW:
            break

    # detect edges in the resized, grayscale image and apply template
        # matching to find the template in the image
        edged = resized
        #edged = cv.Canny(resized, 50, 200)
        result = cv.matchTemplate(edged, template, cv.TM_CCOEFF)
        (_, maxVal, _, maxLoc) = cv.minMaxLoc(result)

        # if we have found a new maximum correlation value, then update
		# the bookkeeping variable
        if found is None or maxVal > found[0]:
            found = (maxVal, maxLoc, r)

    # unpack the bookkeeping variable and compute the (x, y) coordinates
    # of the bounding box based on the resized ratio
    (_, maxLoc, r) = found
    (startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r))
    (endX, endY) = (int((maxLoc[0] + tW) * r), int((maxLoc[1] + tH) * r))
    (centreX, centreY) = (int(startX+(endX-startX)/2), int(startY+(endY-startY)/2))

    # draw a bounding box around the detected result and display the image
    # cv.rectangle(img_to_find, (startX, startY), (endX, endY), (0, 0, 255), 2)
    # image = cv.circle(img_to_find, (sourceX, sourceY), radius=10, color=(0, 0, 255), thickness=-1)
    # image = rescaleFrame(image, scale=0.5)
    # cv.imshow("Image", image)
    # cv.waitKey(0)

    return (centreX, centreY)




def draw_objects_in_Virtual_Env(src):
    """return a Virtual Enviroment with the objects in the Enviroment as perfect polygons"""

    #https://docs.opencv.org/3.4/de/d62/tutorial_bounding_rotated_ellipses.html
    
    
    threshold = 32
    maxX,maxY,_ = src.shape
    print("MAXX,MAXY:",maxX,"\n",maxY,"\n")
    src_gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
    src_gray = cv.blur(src_gray, (3,3))
    max_thresh = 255
    
    canny_output = cv.Canny(src_gray, threshold, threshold * 2)
    
    
    contours, _ = cv.findContours(canny_output, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
    # Find the rotated rectangles and ellipses for each contour
    minRect = [None]*len(contours)
    minEllipse = [None]*len(contours)
    for i, c in enumerate(contours):
        minRect[i] = cv.minAreaRect(c)
        if c.shape[0] > 5:
            minEllipse[i] = cv.fitEllipse(c)
    # Draw contours + rotated rects + ellipses
    
    drawing = np.ones((canny_output.shape[0], canny_output.shape[1], 3), dtype=np.uint8)*255
    
    for i, c in enumerate(contours):

        box = cv.boxPoints(minRect[i])

        box = np.intp(box) #np.intp: Integer used for indexing (same as C ssize_t; normally either int32 or int64)
        print("\nBOX:",box)

        centroid = poly.get_poly_centroid(box,maxY,maxX)
        bgr_poly = src[int(centroid[1]),int(centroid[0])]

        color = (0,0,0)

        if(np.linalg.norm(np.array(bgr_poly)-np.array([40,30,30])) >= 25):
            if(np.linalg.norm(np.array(bgr_poly)-np.array([143,136,105])) <= 25):
                print("sink")
                color = (255,0,0)

        cv.drawContours(drawing, [box], 0, color,-1)
    
    return drawing



def gen_QR(image, filename):
    """generate QR code"""

    data = cv.imread(image)
    data = rescaleFrame(data,0.1)
    QR_img = qrcode.make(data)
    QR_img.save(filename)

    image = cv.imread(filename)
    image = rescaleFrame(image,0.2)
    cv.imwrite(filename,image)

#gen_QR("data/source_front.png","data/source_front_QR.png")


# def find_qr_code(image):
#     original = image.copy()
#     gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
#     blur = cv.GaussianBlur(gray, (9,9), 0)
#     thresh = cv.threshold(blur, 0, 255, cv.THRESH_BINARY_INV + cv.THRESH_OTSU)[1]

#     # Morph close
#     kernel = cv.getStructuringElement(cv.MORPH_RECT, (5,5))
#     close = cv.morphologyEx(thresh, cv.MORPH_CLOSE, kernel, iterations=2)

#     # Find contours and filter for QR code
#     cnts,_ = cv.findContours(close, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
#     minRect = [None]*len(cnts)
#     for i, c in enumerate(cnts):
#         minRect[i] = cv.minAreaRect(c)

#     for i,c in enumerate(cnts):
#         peri = cv.arcLength(c, True)
#         approx = cv.approxPolyDP(c, 0.04 * peri, True)

#         box = cv.boxPoints(minRect[i])

#         box = np.intp(box)


#         x,y,w,h = cv.boundingRect(approx)
#         area = cv.contourArea(c)
#         ar = w / float(h)
#         print(ar)
#         print(area)
#         if len(approx) == 4 and area > 1000 and (ar > .85 and ar < 1.3):
#             ROI = original[y:y+h, x:x+w]
#             ROI = rescaleFrame(ROI,3)
#             QR_data = scan_qr_code(ROI)
#             QR_data = QR_data[4:14]
#             QR_data = list(QR_data.split(" "))
#             QR_data = [int(x) for x in QR_data if x != '']
#             print(QR_data)
#             if(QR_data is not None):
#                 cv.drawContours(image, [box], 0,(36,255,12), 3)
#                 centroid = poly.get_poly_centroid(box,1000,1000)
#                 if QR_data == [0,0,255]:
#                     source_front = [centroid[1],centroid[0]]
#                 if QR_data == [0,255,0]:
#                     source_back = [centroid[1],centroid[0]]
    

#     orientation = np.arctan2((source_front[0]-source_back[0]),(source_front[1]-source_back[1]))
    

#     mass_center = [int((source_front[0]+source_back[0])/2),int((source_front[1]+source_back[1])/2)]

#     # print("ORINTATION:", orientation)


#     # cv.circle(image, (int(source_back[1]), int(source_back[0])), radius=5, color=(255, 0, 255), thickness=-1)

#     # cv.circle(image, (int(source_front[1]), int(source_front[0])), radius=5, color=(255, 0, 0), thickness=-1)
    
#     # cv.circle(image, (int(mass_center[1]), int(mass_center[0])), radius=5, color=(255, 0, 255), thickness=-1)

#     # cv.imshow('thresh', thresh)
#     # cv.imshow('close', close)
#     # image = rescaleFrame(image,0.8)
#     # cv.imshow('image', image)
#     # cv.imshow('ROI', ROI)
#     # cv.waitKey(0)
#     # cv.destroyAllWindows()
#     return orientation, mass_center


# gen_QR("data/source_front.png","data/source_front_QR2.png")

# image = cv.imread("data/find_QR.png")
# find_qr_code(image)

# data = scan_qr_code(image)
# print(data)

# cv.imshow('frame', img2)

# cv.waitKey(0)
# cv.destroyAllWindows()


  
# # define a video capture object
# vid = cv.VideoCapture(2)
  
# while(True):
      
#     # Capture the video frame
#     # by frame
#     ret, frame = vid.read()
  
#     # Display the resulting frame
#     cv.imshow('frame', frame)
      
#     # the 'q' button is set as the
#     # quitting button you may use any
#     # desired button of your choice
#     if cv.waitKey(1) & 0xFF == ord('q'):
#         break
  
# frame = frame[100:399,55:575]
# cv.imwrite("data/Real_Env2.jpeg", frame)
# # After the loop release the cap object
# vid.release()
# # Destroy all the windows
# cv.destroyAllWindows()