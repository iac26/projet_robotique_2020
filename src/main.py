import cv2
import numpy as np
import sys

sys.path.append("vision")
sys.path.append("filtrage")
sys.path.append("globalNavigation")
sys.path.append("localNavigation")
import vision
import globalNavigation as gn



#initialisation


#video capture object
cap = cv2.VideoCapture(0)

#get one frame

ret, frame = cap.read()


scale = vision.detect_scale(frame)

obstacles, ret = vision.detect_obstacles(frame, scale)

targets, ret = vision.detect_targets(frame, scale)







