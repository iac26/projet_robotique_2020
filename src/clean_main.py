# -*- coding: utf-8 -*-
"""
Created on Thu Nov 26 12:38:23 2020

@author: Iacopo
"""

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
import Extended_Kalman_Filter
import start_thymio





#===== INITIALISATION =====
try:
    #cap = cv2.VideoCapture(1)
    pass
except:
    cap = None
    

observer = vision.Observer(None)
observer.stabilize(20)
frame = cv2.imread("vision/images/colors.png")
observer.set_frame(frame)


#TODO: AVERAGED ROBOT POS
#observer.capture()
robot_pos = observer.find_robot()
observer.find_scale()
print(observer.get_scale())
obstacles = observer.find_obstacles()
targets = observer.find_targets()
robot_pos = observer.find_robot()


####
plt.figure()
plt.gca().invert_yaxis()
globalNavigation.printGlobalNavigation(observer.get_obstacles_original(), obstacles, interestPoints = targets)


visibilityGraph, possibleDisplacement = globalNavigation.computeVisibilityGraph(obstacles)
targets.insert(0, [robot_pos[0][0], robot_pos[0][1]]) # the initial position of the Thymio is the starting point of the trajectory

#CHECK THAT NO POINTS ARE IN OBSTACLES
pointsInObstacle = globalNavigation.InterestPointInObstacle(targets, visibilityGraph) 
while pointsInObstacle:
    print("One of the interest point is in a dilated obstacle, please replace them !")
    targets.clear()
    observer.capture()
    targets = observer.find_targets()
    robot_pos = observer.find_robot()
    targets.insert(0, [robot_pos[0][0], robot_pos[0][1]])
    pointsInObstacle = globalNavigation.InterestPointInObstacle(targets, visibilityGraph)
    time.sleep(0.5)


trajectory = globalNavigation.computeTrajectory(visibilityGraph, targets)


final = observer.debug_output(trajectory)
plt.imshow(cv2.cvtColor(final, cv2.COLOR_BGR2RGB))

plt.figure()
plt.gca().invert_yaxis()
globalNavigation.printGlobalNavigation(observer.get_obstacles_original(), obstacles, interestPoints = targets, trajectory = trajectory)


kalman = Extended_Kalman_Filter.Kalman(robot_pos)

start_thymio.connexion_thymio()

value_proximity=[]
value_acceleration=[]
value_speed=[]
actual_position=[0,0]
actual_angle=0
count_trajectory=1
goal_actual=trajectory[count_trajectory]
nb_goal=len(trajectory)
robot_pos = []






last_time = time.time()

while 1:
    time.sleep(0.1)
    
    value_proximity,value_acceleration,value_speed = start_thymio.measure_sensor()
    
    observer.capture()
    
    robot_pos = observer.find_robot()
    
    dt = time.time()-last_time
    last_time = time.time()
    kalman.update_measurements(robot_pos, value_speed)
    estimated_robot_pos = kalman.estimate(dt)

    pxerr = estimated_robot_pos[0][0]-robot_pos[0][0]
    pyerr = estimated_robot_pos[0][1]-robot_pos[0][1]
    aerr = estimated_robot_pos[1]-robot_pos[1]
    
    observer.set_text("dt: {}".format(dt))
    
    
    actual_position,actual_angle = start_thymio.get_position(estimated_robot_pos)
    
    
    if start_thymio.detect_trajectory(actual_position,goal_actual) and count_trajectory < nb_goal-1:         # upload goal 
        count_trajectory+=1
        goal_actual=trajectory[count_trajectory]
       # print("goal has just changed, actual goal is :",goal_actual) 
        start_thymio.follow_the_way_to_dream(actual_position,goal_actual,actual_angle)
        

    elif start_thymio.detect_trajectory(actual_position,goal_actual) and count_trajectory == nb_goal-1:     # if all points are finished
        start_thymio.mission_accomplished()
        break

    else:
        start_thymio.follow_the_way_to_dream(actual_position,goal_actual,actual_angle)    
      

    final = observer.debug_output(trajectory, estimated_robot_pos)
    cv2.imshow('frame',final)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    

start_thymio.stop()

time.sleep(0.5)

start_thymio.deconnexion_thymio()

try:
    cap.release()
    
except:
    pass

cv2.destroyAllWindows()
    
    
    
    
    
    
    
    
    
    
    