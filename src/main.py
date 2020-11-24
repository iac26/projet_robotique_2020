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
    time.sleep(0.1)

#get one frame
ret, frame = cap.read()

frame = cv2.imread("vision/images/colors.png")


plt.imshow(frame)



scale = vision.detect_scale(frame)

dilated_obstacles, obstacles, ret = vision.detect_obstacles(frame, scale)
print(dilated_obstacles)

targets, ret = vision.detect_targets(frame, scale)
#print(targets)

while True:

    robot_pos, ret = vision.detect_robot(frame, scale)
    
    if robot_pos[2]:
        break
    ret, frame = cap.read()

print("robot pos: ", robot_pos)


# ------------------------------------------------- GLOBAL NAVIGATION --------------------------------------------

#dilatedObstacles = globalNavigation.dilateObstacles(obstacles, scalingFactor = 1.8) ########## scaling
#print(dilatedObstacles)

visibilityGraph, possibleDisplacement = globalNavigation.computeVisibilityGraph(dilated_obstacles)

targets.insert(0, [robot_pos[0][0], robot_pos[0][1]]) # the initial position of the Thymio is the starting point of the trajectory

trajectory = globalNavigation.computeTrajectory(visibilityGraph, targets)
print("traject: ",trajectory)


final = vision.debug_output(frame, robot_pos, targets, dilated_obstacles, trajectory, scale)

plt.figure()
plt.imshow(cv2.cvtColor(final, cv2.COLOR_BGR2RGB))


plt.figure()
plt.gca().invert_yaxis()
globalNavigation.printGlobalNavigation(obstacles, dilated_obstacles, interestPoints = targets, trajectory = trajectory)

# ------------------------------------------------- CONNEXION --------------------------------------------------
start_thymio.connexion_thymio()
# ------------------------------------------------- VARIABLE INIT --------------------------------------------------
value_proximity=[]
value_acceleration=[]
value_speed=[]
actual_position=[0,0]
actual_angle=0
count_trajectory=1
goal_actual=trajectory[count_trajectory]
nb_goal=len(trajectory)

# ------------------------------------------------- UPLOAD VARIABLE  --------------------------------------------------


# ------------------------------------------------- THREAD INIT --------------------------------------------------

rt = start_thymio.RepeatedTimer(0.05, start_thymio.measure_sensor)

robot_pos = []

# ----------------------------------------------------------------------------------------------------------------
#cap = cv2.VideoCapture(0)
while True:

    #condition de fin
    time.sleep(0.1)
    value_proximity,value_acceleration,value_speed=start_thymio.get_sensor_value()
    ret, frame = cap.read()
    actual_position,actual_angle=start_thymio.get_position(frame) # upload data in global variables
    #print("actuel position est:",actual_position)
    #print("actuel angle est:",actual_angle)
    #start_thymio.follow_the_way_to_dream(actual_position,goal,actual_angle)
    start_thymio.detect_obstacles()
    if start_thymio.detect_trajectory(actual_position,goal_actual) and count_trajectory < nb_goal-1:         # upload goal 
        count_trajectory+=1
        goal_actual=trajectory[count_trajectory]
        print("goal has just changed, actual goal is :",goal_actual)
        start_thymio.change_mode(goal_actual)
        #start_thymio.follow_the_way_to_dream(actual_position,goal_actual,actual_angle)
        #start_thymio.local_avoidance (actual_position,goal_actual,actual_angle)

    elif start_thymio.detect_trajectory(actual_position,goal_actual) and count_trajectory == nb_goal-1:     # if all points are finished
        start_thymio.mission_accomplished()

    else:
        #start_thymio.local_avoidance (actual_position,goal_actual,actual_angle)
        #start_thymio.follow_the_way_to_dream(actual_position,goal_actual,actual_angle)    
       # print("actual pos: ",actual_position)    
       # print("goal is :",goal_actual)  
        start_thymio.change_mode(goal_actual)

    final = vision.debug_output(frame, robot_pos, targets, obstacles, trajectory, scale)
    cv2.imshow('frame',final)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
                       


cap.release()
cv2.destroyAllWindows()
