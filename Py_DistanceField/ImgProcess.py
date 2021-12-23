import numpy as np
import cv2
from PyToolKit.Vision.Geometry import bilinearInterpolate


def imgFindBoundary(img):

    shapeMask = img != 255

    edgeKenrel = np.array((
        [1, 1, 1],
        [1, 1, 1],
        [1, 1, 1]), dtype="float32") / 9


    dst = cv2.filter2D(img, -1, edgeKenrel)

    boundary = np.logical_and(dst!=0, shapeMask)

    boundaryCoords = np.where(boundary)

    boundaryCoords = np.stack([boundaryCoords[1], boundaryCoords[0]], axis=-1)

    return (255*boundary.astype(np.float32)).astype(np.uint8), boundaryCoords

def initalizeDistanceField(img, boundaryCoords):
    disField = np.zeros(img.shape)
    gradientField = np.zeros((*img.shape, 2))

    shapeMask = img != 255
    shapeCoords = np.where(shapeMask)
    shapeCoords = np.stack([shapeCoords[1], shapeCoords[0]], axis=-1)

    for i in range(shapeCoords.shape[0]):
        disToBounds = boundaryCoords - shapeCoords[i, :]

        dis = np.sqrt(disToBounds[:,0]**2 + disToBounds[:, 1]**2)
        closestBoundaryPoint = np.argmin(dis)

        disField[shapeCoords[i, 1], shapeCoords[i, 0]] = dis[closestBoundaryPoint]

        gradientField[shapeCoords[i, 1], shapeCoords[i, 0], :] = boundaryCoords[closestBoundaryPoint] - shapeCoords[i, :]
        gradientField[shapeCoords[i, 1], shapeCoords[i, 0], :] = gradientField[shapeCoords[i, 1], shapeCoords[i, 0], :] / np.linalg.norm(gradientField[shapeCoords[i, 1], shapeCoords[i, 0], :])


    return disField, gradientField

def getClosestPointInterpolation(pt, disField, gradientField):
    dis = bilinearInterpolate(disField, pt[0], pt[1])
    dir =  bilinearInterpolate(gradientField, pt[0], pt[1])

    closestPt = np.array(pt) + dis * dir
    return closestPt


def getClosestPointBrutalSearch(pt, boundaryCoords):
    disToBounds = boundaryCoords - pt

    dis = np.sqrt(disToBounds[:, 0] ** 2 + disToBounds[:, 1] ** 2)
    closestBoundaryPoint = np.argmin(dis)

    return boundaryCoords[closestBoundaryPoint, :]
