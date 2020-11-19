import cv2
import numpy as np


#returns an array [numpy array pos, angle, visible T/F]
def detect_robot(frame, scale=1):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    blue_MIN = np.array([83, 110, 110], np.uint8)
    blue_MAX = np.array([120, 255, 255], np.uint8)
            
    #Using inRange to find the desired range
    mask = cv2.inRange(hsv,  blue_MIN, blue_MAX)
    
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL  , cv2.CHAIN_APPROX_SIMPLE)
    
    #clean contours
    
    AREA_THRESH = 500
    
    MERGE_THRESH = 0.04
    
    good_cnt = []
    
    for cnt in contours:
        # only take big enough contours
        if (cv2.contourArea(cnt) >= AREA_THRESH):
            #convex hull
            #hull = cv2.convexHull(cnt)
            hull = cnt
            #lower poly approx
            epsilon = MERGE_THRESH*cv2.arcLength(hull,True)
            approx = cv2.approxPolyDP(hull,epsilon,True)
            if(len(approx) == 3):
                K = 2
                A = 0
                B = 0
                C = 0
                dAB = 0
                dBC = 0
                dCA = 0
                p1 = approx[0][0]
                p2 = approx[1][0]
                p3 = approx[2][0]
                d1 = np.linalg.norm(p2-p1)
                d2 = np.linalg.norm(p3-p2)
                d3 = np.linalg.norm(p1-p3)
                min_ix = np.argmin([d1, d2, d3])
                if(min_ix == 0):
                    A = p3
                    B = p2
                    C = p1
                    dAB = d2
                    dBC = d1
                    dCA = d3
                elif(min_ix == 1):
                    A = p1
                    B = p3
                    C = p2
                    dAB = d3
                    dBC = d2
                    dCA = d1
                else:
                    A = p2
                    B = p3
                    C = p1
                    dAB = d2
                    dBC = d3
                    dCA = d1
                score = abs(dAB-dCA)+abs(K*dBC - dAB)+abs(K*dBC - dCA)

                good_cnt.append([A, B, C, score]);
                       
    good_cnt = sorted(good_cnt, key = lambda x: x[3])
    
    robot_pos = [np.array([0, 0]), 0, False];
    
    if(len(good_cnt) > 0):
        robot_visible = True
        A = good_cnt[0][0]
        B = good_cnt[0][1]
        C = good_cnt[0][2]
        D = (np.mean([[B, C]], axis=1))[0]
        
        
        Center = (np.mean([[A, B, C]], axis=1))[0]
        
        
        

        
        direction = A - D
        
        angle = np.arctan2(direction[1], direction[0])
        
        frame = cv2.line(frame, (int(D[0]), int(D[1])), (int(A[0]), int(A[1])), color=(0, 0, 255), thickness=1)
        frame = cv2.circle(frame, (int(Center[0]), int(Center[1])), radius=5, color=(0, 0, 255), thickness=-1)
        Center = Center*scale
        text =  "position: ({:0.2f}, {:0.2f}) angle: {:0.4f}".format(Center[0], Center[1], angle)
        font = cv2.FONT_HERSHEY_SIMPLEX 
        cv2.putText(frame, text, (10, 50), font, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        
        
        robot_pos = [Center, angle, True]
    return robot_pos, frame




def detect_obstacles(frame, scale=1):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    

    red_MIN = np.array([0, 119, 159], np.uint8)
    red_MAX = np.array([10, 217, 255], np.uint8)

            
    #Using inRange to find the desired range
    mask = cv2.inRange(hsv,  red_MIN, red_MAX)
    
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL  , cv2.CHAIN_APPROX_SIMPLE)
    
    #clean contours
    
    AREA_THRESH = 500
    
    MERGE_THRESH = 0.04
    
    clean_contours = []
    
    for cnt in contours:
        # only take big enough contours
        if (cv2.contourArea(cnt) >= AREA_THRESH):
            #convex hull
            #hull = cv2.convexHull(cnt)
            hull = cnt
            #lower poly approx
            epsilon = MERGE_THRESH*cv2.arcLength(hull,True)
            approx = cv2.approxPolyDP(hull,epsilon,True)
            
            clean_contours.append(approx)
            
    
    cv2.drawContours(frame, clean_contours, -1, (0,255,0), 3) 
    for cnt in clean_contours:
        for pt in cnt:
            frame = cv2.circle(frame, (pt[0][0], pt[0][1]), radius=5, color=(0, 0, 255), thickness=-1)
    return clean_contours, frame


def detect_targets(frame, scale=1):
    return np.array([[]]), frame


def detect_scale(frame):
    return 1

