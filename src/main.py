import cv2
import numpy as np
import matplotlib.pyplot as plt
import sys
import time

sys.path.append("vision")
sys.path.append("filtrage")
sys.path.append("globalNavigation")
sys.path.append("localNavigation")

import vision
import globalNavigation


# -------------------------------------------------- VISION INIT --------------------------------------------------

#video capture object
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

#read a few frames to allow the camera to adjust
for i in range(20):
    ret, frame = cap.read()

#get one frame
ret, frame = cap.read()

#frame = cv2.imread("vision/images/colors.png")

scale = vision.detect_scale(frame)

obstacles, ret = vision.detect_obstacles(frame, scale)
print(obstacles)

targets, ret = vision.detect_targets(frame, scale)
print(targets)

robot_pos, ret = vision.detect_robot(frame, scale)

final = vision.debug_output(frame, robot_pos, targets, obstacles, scale)

plt.figure()
plt.imshow(cv2.cvtColor(final, cv2.COLOR_BGR2RGB))


# ------------------------------------------------- GLOBAL NAVIGATION --------------------------------------------

dilatedObstacles = globalNavigation.dilateObstacles(obstacles, scalingFactor = 1.8) ########## scaling
print(dilatedObstacles)

visibilityGraph, possibleDisplacement = globalNavigation.computeVisibilityGraph(dilatedObstacles)

targets.insert(0, [robot_pos[0][0], robot_pos[0][1]]) # the initial position of the Thymio is the starting point of the trajectory

trajectory = globalNavigation.computeTrajectory(visibilityGraph, targets)

plt.figure()
plt.gca().invert_yaxis()
globalNavigation.printGlobalNavigation(obstacles, dilatedObstacles, interestPoints = targets, trajectory = trajectory)

# ------------------------------------------------- THREAD INIT --------------------------------------------------



# ----------------------------------------------------------------------------------------------------------------

while True:

    #condition de fin
    time.sleep(0.1)


cap.release()
