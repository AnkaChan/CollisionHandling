import cv2
import numpy as np

from ImgProcess import *


# define the events for the
# mouse_click.
def mouse_click(event, x, y,
                flags, param):
    # to check if left mouse
    # button was clicked
    if event == cv2.EVENT_LBUTTONDOWN:
        # font for left click event
        pt = np.array([x, y])
        closestPt = getClosestPointInterpolation(pt, disField, gradientField)
        closestPtGd = getClosestPointBrutalSearch(pt, boundaryCoords)

        imgCopy = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        cv2.line(imgCopy, pt, closestPt.astype(int), (0,255,255), 3)
        imgCopy2 = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        cv2.line(imgCopy2, pt, closestPtGd.astype(int), (0,255,0), 3)

        cv2.imshow('ClosestPointInterpolation', imgCopy)
        cv2.imshow('ClosestPointGroundTruth', imgCopy2)

if __name__ == '__main__':
    inFile = r'Data\Shape1HighRes.png'

    img = cv2.imread(inFile, cv2.IMREAD_GRAYSCALE)

    img[img>150] = 255
    img[img<=150] = 0

    boundary, boundaryCoords = imgFindBoundary(img)

    disField, gradientField = initalizeDistanceField(img, boundaryCoords)

    # cv2.imshow('Shape', img)
    # cv2.imshow('boundary', boundary)
    # cv2.imshow('disField', (255*disField/np.max(disField)).astype(np.uint8))

    pt = np.array([134, 173, ])

    closestPt = getClosestPointInterpolation(pt, disField, gradientField)
    closestPtGd = getClosestPointBrutalSearch(pt, boundaryCoords)
    # imgCopy = np.deepcopy(img)

    imgCopy = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    imgCopy2 = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    cv2.imshow('ClosestPointInterpolation', imgCopy)
    cv2.imshow('ClosestPointGroundTruth', imgCopy2)


    cv2.setMouseCallback('ClosestPointInterpolation', mouse_click)

    cv2.waitKey()
