# IMPORT
# pip install pyvisgraph
import math
import cv2
import numpy as np
import pyvisgraph as vg
import matplotlib.pyplot as plt

# Define
X = 0
Y = 1


def computeVisibilityGraph(contoursMapped):
    """Given the dilated obstacles, compute the visibility graph

    Parameters
    ----------
    contoursMapped : list of list of list
        There are several dilated obstacles
        Each one has several extremities
        Each extremity has (x, y) coordinates

    Returns
    -------
    g : object of class Graph
        the visibility graph of our problem

    possibleDisplacement : dictionary
        key : tuple containing (x,y) coordinates of one of the edges of the dilated obstacles
        value : list of all other edges visible from the key edge
    """

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

    return g, possibleDisplacement


def InterestPointInObstacle(interestPoints, graph):
    """Given the visibility graph and the points of interest
       Says if some interest points are in the dilated obstacles or not

    Parameters
    ----------
    interestPoints : list of list
        Each point of interest has (x, y) coordinates, i.e locations where the thymio need to go

    graph : object of class Graph
        the visibility graph of our problem

    Returns
    -------
    a boolean :
        True if some point of interest are located in the dilated obstacles
        False if none of the interest points are located in the dilated obstacles
    """

    for point in interestPoints:
        if graph.point_in_polygon(vg.Point(point[X], point[Y])) != -1:
            return True

    return False


def computeTrajectory(graph, interestPoints): 
    """Given the visibility graph and the points of interest
       Compute a path planning allowing the Thymio to pass through all the point of interest
       and come back to his initial position

    Parameters
    ----------
    g : object of class Graph
        the visibility graph of our problem

    Returns
    -------
    path : list of list
        Each point defining the trajectory has (x, y) coordinates
        The final trajectory is basically a line joining all point of the path
    """
    
    startingPoint = interestPoints[0]
    pointTravelled2 = [startingPoint]
    path = [startingPoint]
    interestPointsLeft = [x for x in interestPoints if x != startingPoint]
    i = 0

    while i != len(interestPoints):

        index = -1
        minimum = np.inf
        point = pointTravelled2[i]

        # find the closest interest point to the current interest point
        if i != len(interestPoints) - 1:
            for pointLeft, j in zip(interestPointsLeft, range(len(interestPointsLeft))):
                dist = math.sqrt((point[X] - pointLeft[X])**2 + (point[Y] - pointLeft[Y])**2)
                if dist < minimum:
                    minimum = dist
                    index = j

        # if there is no remaining point of interest, we need to come back to the starting point
        else:
            interestPointsLeft.append(startingPoint)
            index = 0

        # compute an optimal path from the current interest point to his closest interest point using the visibility graph
        # and add the points of this new optimal path in the total path
        shortest = graph.shortest_path(vg.Point(point[X], point[Y]), vg.Point(interestPointsLeft[index][X], interestPointsLeft[index][Y]))
        for j in range(1, len(shortest)):
            path.append([shortest[j].x, shortest[j].y])

        # remove the closest interest point as we finish to explore it
        pointTravelled2.append([interestPointsLeft[index][X], interestPointsLeft[index][Y]])
        interestPointsLeft.remove([interestPointsLeft[index][X], interestPointsLeft[index][Y]])

        i += 1

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

    for obstacleOriginal in contours:
        for extremityOriginal in obstacleOriginal:
            xOriginal.append(extremityOriginal[X])
            yOriginal.append(extremityOriginal[Y])

        xOriginal.append(obstacleOriginal[0][X])
        yOriginal.append(obstacleOriginal[0][Y])

        plt.plot(xOriginal, yOriginal, 'b')

        xOriginal.clear()
        yOriginal.clear()

    
    for obstacleDilated in contoursMapped:
        for extremityDilated in obstacleDilated:
            xDilated.append(extremityDilated[X])
            yDilated.append(extremityDilated[Y])

        xDilated.append(obstacleDilated[0][X])
        yDilated.append(obstacleDilated[0][Y])

        plt.plot(xDilated, yDilated, 'm')

        xDilated.clear()
        yDilated.clear()


    if possibleDisplacement:
        for extremity in possibleDisplacement:
            for visiblePoint in possibleDisplacement[extremity]:
                plt.plot([extremity[X], visiblePoint[X]], [extremity[Y], visiblePoint[Y]], 'm')

    
    if interestPoints:
        for point in interestPoints:
            plt.plot([point[X]], [point[Y]], 'kx', markersize=12)


    if trajectory:
        for i in range (1, len(trajectory)):
            plt.arrow(trajectory[i-1][X], trajectory[i-1][Y], trajectory[i][X] - trajectory[i-1][X], trajectory[i][Y] - trajectory[i-1][Y], head_width=8, length_includes_head=True, color  = 'k', width = 2)

    plt.show()
