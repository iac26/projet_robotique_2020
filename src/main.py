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
import start_thymio


# -------------------------------------------------- VISION INIT --------------------------------------------------

#video capture object
cap = cv2.VideoCapture(0)

#read a few frames to allow the camera to adjust
for i in range(20):
    ret, frame = cap.read()

#get one frame
ret, frame = cap.read()

frame = cv2.imread("vision/images/colors.png")
#sizeframe

scale = vision.detect_scale(frame)

obstacles, ret = vision.detect_obstacles(frame, scale)
print(obstacles)

targets, ret = vision.detect_targets(frame, scale)
#print(targets)

robot_pos, ret = vision.detect_robot(frame, scale)


# ------------------------------------------------- GLOBAL NAVIGATION --------------------------------------------

#dilatedObstacles = globalNavigation.dilateObstacles(obstacles, scalingFactor = 1.8) ########## scaling
#print(dilatedObstacles)

visibilityGraph, possibleDisplacement = globalNavigation.computeVisibilityGraph(obstacles)

targets.insert(0, [robot_pos[0][0], robot_pos[0][1]]) # the initial position of the Thymio is the starting point of the trajectory

trajectory = globalNavigation.computeTrajectory(visibilityGraph, targets)
print("traject: ",trajectory)


final = vision.debug_output(frame, robot_pos, targets, obstacles, trajectory, scale)

plt.figure()
plt.imshow(cv2.cvtColor(final, cv2.COLOR_BGR2RGB))



plt.figure()
plt.gca().invert_yaxis()
globalNavigation.printGlobalNavigation(obstacles, obstacles, interestPoints = targets, trajectory = trajectory)

# ------------------------------------------------- CONNEXION --------------------------------------------------
start_thymio.connexion_thymio()
# ------------------------------------------------- VARIABLE INIT --------------------------------------------------
value_proximity=[]
value_acceleration=[]
value_speed=[]
actual_position=[0,0]
goal=[0,0]
actual_angle=0
goal=trajectory[1]
# ------------------------------------------------- UPLOAD VARIABLE  --------------------------------------------------


# ------------------------------------------------- THREAD INIT --------------------------------------------------

rt = start_thymio.RepeatedTimer(0.05, start_thymio.measure_sensor)



# ----------------------------------------------------------------------------------------------------------------
#cap = cv2.VideoCapture(0)
while True:

    #condition de fin
    time.sleep(0.1)
    value_proximity,value_acceleration,value_speed=start_thymio.get_sensor_value()
    actual_position,actual_angle=start_thymio.get_position(cap) # upload data in global variables
    #print("actuel position est:",actual_position)
    #print("actuel angle est:",actual_angle)
    start_thymio.follow_the_way_to_dream(actual_position,goal,actual_angle)

cap.release()
