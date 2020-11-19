sys.path.append("vision")
sys.path.append("filtrage")
sys.path.append("globalNavigation")
sys.path.append("localNavigation")

import cv2
import numpy as np
import matplotlib.pyplot as plt
import sys
import time

import vision
import globalNavigation


#video capture object
cap = cv2.VideoCapture(0)

#read a few frames to allow the camera to adjust

for i in range(20):
    ret, frame = cap.read()

#get one frame

ret, frame = cap.read()

#frame = cv2.imread("vision/images/colors.png")

scale = vision.detect_scale(frame)

obstacles, ret = vision.detect_obstacles(frame, scale)

targets, ret = vision.detect_targets(frame, scale)

robot_pos, ret = vision.detect_robot(frame, scale)


#final = vision.debug_output(frame, robot_pos, targets, obstacles, scale)

#plt.imshow(cv2.cvtColor(final, cv2.COLOR_BGR2RGB))


#lancer les threads


while True:
    #condition de fin
    time.sleep(0.1)

cap.release()

    


