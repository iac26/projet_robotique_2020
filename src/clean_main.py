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
import start_thymio




if __name__ == "__main__":

#===== INITIALISATION =====
    try:
        cap =cv2.VideoCapture(0)
    except:
        cap = None
        
    
    observer = vision.Observer(cap)
    observer.stabilize(20)
    frame = cv2.imread("vision/images/colors.png")
    observer.set_frame(frame)
    observer.find_scale()
    obstacles = observer.find_obstacles()
    targets = observer.find_targets()
    robot_pos = observer.find_robot()
    
    visibilityGraph, possibleDisplacement = globalNavigation.computeVisibilityGraph(obstacles)
    targets.insert(0, [robot_pos[0][0], robot_pos[0][1]]) # the initial position of the Thymio is the starting point of the trajectory
    trajectory = globalNavigation.computeTrajectory(visibilityGraph, targets)

    print(observer.get_error_log())
    
    final = observer.debug_output(trajectory)
    plt.imshow(cv2.cvtColor(final, cv2.COLOR_BGR2RGB))
    
    plt.figure()
    plt.gca().invert_yaxis()
    globalNavigation.printGlobalNavigation(observer.get_obstacles_original(), obstacles, interestPoints = targets, trajectory = trajectory)
    
    try:
        cap.release()
    except:
        pass

    
    
    
    
    
    
    
    
    
    
    