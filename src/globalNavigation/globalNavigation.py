# IMPORT
import cv2 as cv
import numpy as np
import pyvisgraph as vg
import matplotlib.pyplot as plt

# Define
X = 0
Y = 1

# CARE SI OBSTACLES TROP PROCHES LA DILATATION VA FAIRE RENTRER DEUX OBSTACLES ENTRE EUX !!

def dilateObstacles (contours):
    """Given the contours of obstacles, dilate these contours 

    Parameters
    ----------
    contours : list of list of list
        The camera detect several obstacles
        Each obstacle has several extremities
        Each extremity has (x, y) coordinates

    Returns
    -------
    contoursMapped : list of list of list
        Same structure as contours, each extremity's coordinate has been dilated
    """

    # Compute centroid of each obstacle
    centroids = []
    for obstacle in contours:
        M = cv.moments(obstacle)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        centroids.append([cx, cy])


    # For each obstacle, map the original contour points to new coordinates where (0, 0) is the centroid
    contoursMapped = [[] for _ in range(len(contours))]

    for obstacle, i in zip(contours, range(len(contours))):
        for extremity in obstacle:
            contoursMapped[i].append([extremity[0][X] - centroids[i][X], extremity[0][Y] - centroids[i][Y]])


    # Scale position
    scalingFactor = 1.4         # 40 % bigger

    for obstacle, i in zip(contoursMapped, range(len(contoursMapped))):
        for j in range(len(obstacle)):
            contoursMapped[i][j][X] *= scalingFactor
            contoursMapped[i][j][Y] *= scalingFactor


    # Map it back to previous coordinates by adding back position of centroids
    for obstacle, i in zip(contoursMapped, range(len(contoursMapped))):
        for j in range(len(obstacle)):
            contoursMapped[i][j][X] = int(contoursMapped[i][j][X] + centroids[i][X])
            contoursMapped[i][j][Y] = int(contoursMapped[i][j][Y] + centroids[i][Y])

    return contoursMapped


def printGlobalNavigation(contours, contoursMapped, possibleDisplacement = {}):
    """Plot the original contours and the dilated contours using matplotlib
       Plot the visibility graph if possibleDisplacement is given 

    Parameters
    ----------
    contours : list of list of list
        The camera detect several obstacles
        Each obstacle has several extremities
        Each extremity has (x, y) coordinates

    contoursMapped : list of list of list
        Same structure as contours, each extremity's coordinate has been dilated

    possibleDispacement : dictionary
        Each extremity point has several visible points, i.e possible destinations for the Thymio

    Returns
    -------

    """

    xOriginal = []
    yOriginal = []
    xDilated = []
    yDilated = []

    for obstacleOriginal, obstacleDilated in zip(contours, contoursMapped):
        for extremityOriginal, extremityDilated in zip(obstacleOriginal, obstacleDilated):
            xOriginal.append(extremityOriginal[0][X])
            xDilated.append(extremityDilated[X])
            yOriginal.append(extremityOriginal[0][Y])
            yDilated.append(extremityDilated[Y])

        xOriginal.append(obstacleOriginal[0][0][X])
        xDilated.append(obstacleDilated[0][X])
        yOriginal.append(obstacleOriginal[0][0][Y])
        yDilated.append(obstacleDilated[0][Y])

        plt.plot(xOriginal, yOriginal, 'b')
        plt.plot(xDilated, yDilated, 'm')

        xOriginal.clear()
        xDilated.clear()
        yOriginal.clear()
        yDilated.clear()


    if possibleDisplacement:
        for extremity in possibleDisplacement:
            for visiblePoint in possibleDisplacement[extremity]:
                plt.plot([extremity[X], visiblePoint[X]], [extremity[Y], visiblePoint[Y]], 'm')

    plt.show()


# Vision's input
contours = [np.array([[[504, 236]], [[495, 199]], [[380, 212]], [[438, 274]]], dtype=np.int32), 
            np.array([[[170, 195]], [[254, 275]], [[296, 238]], [[235, 194]]], dtype=np.int32), 
            np.array([[[302, 168]], [[290, 182]], [[294, 199]], [[312, 209]], [[333, 203]], [[337, 175]]], dtype=np.int32), 
            np.array([[[228, 151]], [[301, 102]], [[219,  89]]], dtype=np.int32), 
            np.array([[[481, 130]], [[457,  66]], [[360,  81]], [[434, 150]]], dtype=np.int32)]


# Dilate obstacles and print them
contoursMapped = dilateObstacles(contours)
printGlobalNavigation(contours, contoursMapped)


# Compute the visibility graph
polys = [[] for _ in contoursMapped]

for obstacle, i in zip(contoursMapped, range(len(contoursMapped))):
    for extremity in obstacle:
        polys[i].append(vg.Point(extremity[X], extremity[Y]))

g = vg.VisGraph()
g.build(polys)


# Create a dictionary where each extremety point has several visible points i.e possible destinations
possibleDisplacement = {}

for obstacle, i in zip(contoursMapped, range(len(contoursMapped))):
    for extremity, j in zip(obstacle, range(len(obstacle))):
        visible = g.find_visible(polys[i][j])
        possibleDisplacement[(extremity[X], extremity[Y])] = [[point.x, point.y] for point in visible]
        visible.clear()

#print(possibleDisplacement)
printGlobalNavigation(contours, contoursMapped, possibleDisplacement)
