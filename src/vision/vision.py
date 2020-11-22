import cv2
import numpy as np


def find_color(frame, hsv_low, hsv_high):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    

    #Using inRange to find the desired range
    mask = cv2.inRange(hsv,  hsv_low, hsv_high)
    
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
    return clean_contours
    
    
    


#returns an array [numpy array pos, angle, visible T/F]
def detect_robot(frame, scale=1):
    blue_low = np.array([83, 110, 110], np.uint8)
    blue_high = np.array([120, 255, 255], np.uint8)
    frame = frame.copy()
    
    clean_contours = find_color(frame, blue_low, blue_high)
    
    good_cnt = []
    
    for cnt in clean_contours:
        if(len(cnt) == 3):
            K = 2
            A = 0
            B = 0
            C = 0
            dAB = 0
            dBC = 0
            dCA = 0
            p1 = cnt[0][0]
            p2 = cnt[1][0]
            p3 = cnt[2][0]
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
    
    robot_pos = [np.array([0, 0]), 0, False, 0];
    
    if(len(good_cnt) > 0):
        robot_visible = True
        A = good_cnt[0][0]
        B = good_cnt[0][1]
        C = good_cnt[0][2]
        D = (np.mean([[B, C]], axis=1))[0]
        
        
        Center = (np.mean([[A, B, C]], axis=1))[0]

        direction = A - D
        
        size = np.linalg.norm(direction)
        
        angle = np.arctan2(direction[1], direction[0])
        
        frame = cv2.line(frame, (int(D[0]), int(D[1])), (int(A[0]), int(A[1])), color=(0, 0, 255), thickness=1)
        frame = cv2.circle(frame, (int(Center[0]), int(Center[1])), radius=5, color=(0, 0, 255), thickness=-1)
        Center = Center*scale
        text =  "position: ({:0.2f}, {:0.2f}) angle: {:0.4f}".format(Center[0], Center[1], angle)
        font = cv2.FONT_HERSHEY_SIMPLEX 
        cv2.putText(frame, text, (10, 50), font, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        
        
        robot_pos = [Center, angle, True, size]
        
    return robot_pos, frame




def detect_obstacles(frame, scale=1):
    frame = frame.copy()
    red_low = np.array([140, 100, 100], np.uint8)
    red_high = np.array([180, 255, 255], np.uint8)
    
    clean_contours = find_color(frame, red_low, red_high)
            
    
    cv2.drawContours(frame, clean_contours, -1, (0,255,0), 3) 
    for cnt in clean_contours:
        for pt in cnt:
            frame = cv2.circle(frame, (pt[0][0], pt[0][1]), radius=5, color=(0, 0, 255), thickness=-1)
    return np.multiply(clean_contours, scale), frame


def detect_targets(frame, scale=1):
    frame = frame.copy()
    green_low = np.array([40, 50, 0], np.uint8)
    green_high = np.array([85, 255, 150], np.uint8)
    
    clean_contours = find_color(frame, green_low, green_high)
    
    centroids = []
    
    for cnt in clean_contours:
        mom = cv2.moments(cnt)
        if mom["m00"] != 0:
            cx = int(mom["m10"] / mom["m00"])
            cy = int(mom["m01"] / mom["m00"])
            centroids.append([cx, cy])
        else:
            pass
            
        
            
    cv2.drawContours(frame, clean_contours, -1, (0,255,0), 3)
    for pt in centroids:
        frame = cv2.circle(frame, (pt[0], pt[1]), radius=5, color=(0, 0, 255), thickness=-1)
    """np.multiply(centroids, scale)"""
    return centroids, frame



def detect_scale(frame):
    robot_pos, ret = detect_robot(frame)
    
    return 1 # robot_pos[3]/57.5



def debug_output(frame, robot_pos, targets, obstacles, scale):
    frame = frame.copy()
    for pt in targets:
        pt = np.floor(np.divide(pt, scale)).astype(int)
        frame = cv2.circle(frame, (pt[0], pt[1]), radius=5, color=(0, 255, 0), thickness=-1)
        
    for cnt in obstacles:
        for pt in cnt:
            pt = np.floor(np.divide(pt, scale)).astype(int)
            frame = cv2.circle(frame, (pt[0][0], pt[0][1]), radius=5, color=(0, 0, 255), thickness=-1)
    
    pt = np.floor(np.divide(robot_pos[0], scale)).astype(int)
    frame = cv2.circle(frame, (pt[0], pt[1]), radius=5, color=(255, 0, 0), thickness=-1)
    pt2 = pt.copy()
    LEN = 20
    pt2[0] = int(pt[0] + scale*LEN*np.cos(robot_pos[1]))
    pt2[1] = int(pt[1] + scale*LEN*np.sin(robot_pos[1]))
    frame = cv2.line(frame, (pt[0], pt[1]), (pt2[0], pt2[1]), color=(255, 0, 0), thickness=3)
    
    
    return frame
    






