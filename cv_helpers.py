import numpy as np
import imutils
import cv2 as cv


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
    template = rescaleFrame(template_img, scale=0.1)
    template = cv.cvtColor(template, cv.COLOR_BGR2GRAY)
    #template = cv.Canny(template, 50, 200)
    (tH, tW) = template.shape[:2]
    cv.imshow("Template", template)


    gray = cv.cvtColor(img_to_find, cv.COLOR_BGR2GRAY)
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
        #edged = cv.Canny(resized, 50, 200)
        edged = resized
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












# cap = cv.VideoCapture(1 + cv.CAP_DSHOW)

# isTrue, frame = cap.read()


# cv.imshow('frame', gray)

# while (True):
#     ret, frame = cap.read()

#     # frame_resize = rescale(frame, scale=0.5)

#     gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

#     cv.imshow('frame', gray)


#     if cv.waitKey(1) & 0xFF == ord('q'):
#         break

# cv.waitKey(0)

# # cap.release()
# # cv.destroyAllWindows()
