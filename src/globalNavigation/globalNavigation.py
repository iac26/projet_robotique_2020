# IMPORT
import math
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


def computeTrajectory(graph, interestPoints): # BUG VA PAS VERS TOUT LES POINTS
    
    startingPoint = interestPoints[0]
    path = [startingPoint]
    interestPointsLeft = [x for x in interestPoints if x != startingPoint]

    for point in interestPoints:
        minimum = np.inf
        index = -1

        # find the closest interest point to the current interest point
        if point != interestPoints[-1]:
            for pointLeft, j in zip(interestPointsLeft, range(len(interestPointsLeft))):
                dist = math.sqrt((point[X] - pointLeft[X])**2 + (point[Y] - pointLeft[Y])**2)
                if dist < minimum:
                    minimum = dist
                    index = j
        else:
            interestPointsLeft.append(startingPoint)
            index = 0

        # compute an optimal path from the current interest point and his closest interest point using the visibility graph
        # and add the points of this new optimal path in the total path
        shortest = graph.shortest_path(vg.Point(point[X], point[Y]), vg.Point(interestPointsLeft[index][X], interestPointsLeft[index][Y]))
        for j in range(1, len(shortest)):
            path.append([shortest[j].x, shortest[j].y])

        # remove the closest interest point as we finish to explore it
        interestPointsLeft.remove([interestPointsLeft[index][X], interestPointsLeft[index][Y]])

    return path


def printGlobalNavigation(contours, contoursMapped, possibleDisplacement = {}, interestPoints = [], trajectory = []):
    """Plot the original contours and the dilated contours using matplotlib
       Plot the visibility graph if possibleDisplacement is given 
       Plot the Thymio's point of interest if interestPoints is given
       Plot the Thymio's path if the trajectory is given

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

    interestPoints : list of list
        Each point of interest has (x, y) coordinates, i.e locations where the thymio need to go

    trajectory : list of list
        Each point of the trajectory has (x, y) coordinates

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

    
    if interestPoints: # CARE BUG SI ON DONNE PAS EN ARGUMENT
        for point in interestPoints:
            plt.plot([point[X]], [point[Y]], 'kx')


    if trajectory:
        xTrajectory = []
        yTrajectory = []

        for point in trajectory:
            xTrajectory.append(point[X])
            yTrajectory.append(point[Y])

        plt.plot(xTrajectory, yTrajectory, 'g')

    plt.show()


# Vision's input : extremities of each obstacles
contours = [np.array([[[504, 236]], [[495, 199]], [[380, 212]], [[438, 274]]], dtype=np.int32), 
            np.array([[[170, 195]], [[254, 275]], [[296, 238]], [[235, 194]]], dtype=np.int32), 
            np.array([[[302, 168]], [[290, 182]], [[294, 199]], [[312, 209]], [[333, 203]], [[337, 175]]], dtype=np.int32), 
            np.array([[[228, 151]], [[301, 102]], [[219,  89]]], dtype=np.int32), 
            np.array([[[481, 130]], [[457,  66]], [[360,  81]], [[434, 150]]], dtype=np.int32)]

# Vision's input : position of point of interest -> CHECKER SI POINTS DE VISION IN POLYGON
interestPoints = [[149, 286], [319, 272], [277, 151], [496, 171], [508, 69], [347, 52], [202, 77]]


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


# Compute trajectory going through all the points of interest and going back to the starting point
trajectory = computeTrajectory(g, interestPoints)
print(interestPoints)
print(trajectory)
printGlobalNavigation(contours, contoursMapped, possibleDisplacement, interestPoints, trajectory)