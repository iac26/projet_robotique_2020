import cv2
import numpy as np
import matplotlib.pyplot as plt
import time

RED_LOW  = [150, 100, 100]
RED_HIGH = [179, 255, 255]

GREEN_LOW  = [41, 32, 0]
GREEN_HIGH = [77, 140, 140]

BLUE_LOW  = [87, 129, 80]
BLUE_HIGH = [131, 255, 255]


def cleanup_contours(contours):
    #clean contours
    AREA_THRESH = 100
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


def find_color(frame, hsv_low, hsv_high):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    

    #Using inRange to find the desired range
    mask = cv2.inRange(hsv,  hsv_low, hsv_high)
    
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL  , cv2.CHAIN_APPROX_SIMPLE)
    
    
    return cleanup_contours(contours)
    


#returns an array [numpy array pos, angle, visible T/F]
def detect_robot(frame, scale=1):
    blue_low = np.array(BLUE_LOW, np.uint8)
    blue_high = np.array(BLUE_HIGH, np.uint8)
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
    
    robot_pos = [np.array([0, 0]), 0, False, 0]
    
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
        Center = np.multiply(Center, scale).astype(int)
        text =  "position: ({:0.2f}, {:0.2f}) angle: {:0.4f}".format(Center[0], Center[1], angle)
        font = cv2.FONT_HERSHEY_SIMPLEX 
        cv2.putText(frame, text, (10, 50), font, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        
        
        robot_pos = [Center, angle, True, size]
        
    return robot_pos, frame


DIL_COEFF = 40

def detect_obstacles(frame, scale=1):
    frame = frame.copy()
    red_low = np.array(RED_LOW, np.uint8)
    red_high = np.array(RED_HIGH, np.uint8)
    
    clean_contours = find_color(frame, red_low, red_high)
            
    original_contours = []
    dil_contour = []
    for cnt in clean_contours:
        mom = cv2.moments(cnt)
        ncnt = []
        ocnt = []
        if mom["m00"] != 0:
            cx = int(mom["m10"] / mom["m00"])
            cy = int(mom["m01"] / mom["m00"])
            C = np.array([cx, cy])
            for pt in cnt:
                N = pt-C
                N = N/np.linalg.norm(N)
                npt = (pt+DIL_COEFF/scale*N).astype(int)
                ncnt.append(npt)
                ocnt.append(pt[0])
            dil_contour.append(np.array(ncnt))
            original_contours.append(np.multiply(ocnt, scale).astype(int))
        else:
            pass
        
        
    
    cv2.drawContours(frame, clean_contours, -1, (0,255,0), 3)
    
    black = np.zeros(frame.shape[:2], dtype=np.uint8)
    
    for i in range(len(dil_contour)):
        cv2.drawContours(black, dil_contour, i, (255), -1)
        
    
    
    
    #find contours
    
    contours, hierarchy = cv2.findContours(black, cv2.RETR_EXTERNAL  , cv2.CHAIN_APPROX_SIMPLE)
    
    clean_dil_contours = cleanup_contours(contours)

    scaled_contours = []
    for cnt in clean_dil_contours:
        ncnt = []
        for pt in cnt:
            frame = cv2.circle(frame, (pt[0][0], pt[0][1]), radius=5, color=(0, 0, 255), thickness=-1)
            ncnt.append(pt[0])
        scaled_contours.append(np.multiply(ncnt, scale).astype(int))
    
    
    return scaled_contours, original_contours, frame


def detect_targets(frame, scale=1):
    frame = frame.copy()
    green_low = np.array(GREEN_LOW, np.uint8)
    green_high = np.array(GREEN_HIGH, np.uint8)
    
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
    scaled_centroids = []
    
    for pt in centroids:
        frame = cv2.circle(frame, (pt[0], pt[1]), radius=5, color=(0, 0, 255), thickness=-1)
        scaled_centroids.append(np.multiply(pt, scale).astype(int).tolist())
        
    return scaled_centroids, frame


ROBOT_LEN = 100

def detect_scale(frame):
    robot_pos, ret = detect_robot(frame)
    if robot_pos[2]:
        return ROBOT_LEN/robot_pos[3]
    else:
        return 1



def debug_output(frame, robot_pos, targets, obstacles, trajectory, estimated_robot_pos, text, scale):
    frame = frame.copy()
    for pt in targets:
        pt = np.floor(np.divide(pt, scale)).astype(int)
        frame = cv2.circle(frame, (pt[0], pt[1]), radius=5, color=(0, 255, 0), thickness=-1)
        
    for cnt in obstacles:
        for pt in cnt:
            pt = np.floor(np.divide(pt, scale)).astype(int)
            frame = cv2.circle(frame, (pt[0], pt[1]), radius=5, color=(0, 0, 255), thickness=-1)
    
    fst = 0
    
    trajectory = np.divide(trajectory, scale).astype(int)
    for pt in trajectory:
        if fst == 0:
            fst = 1
            lpt = pt  
        else:
            frame = cv2.line(frame, (int(lpt[0]), int(lpt[1])), (int(pt[0]), int(pt[1])), color=(255, 0, 255), thickness=3) 
        lpt = pt

    if(robot_pos[2]):
        pt = np.floor(np.divide(robot_pos[0], scale)).astype(int)
        frame = cv2.circle(frame, (pt[0], pt[1]), radius=5, color=(255, 0, 0), thickness=-1)
        pt2 = pt.copy()
        LEN = 20
        pt2[0] = int(pt[0] + scale*LEN*np.cos(robot_pos[1]))
        pt2[1] = int(pt[1] + scale*LEN*np.sin(robot_pos[1]))
        frame = cv2.line(frame, (pt[0], pt[1]), (pt2[0], pt2[1]), color=(255, 0, 0), thickness=3)
    
    if(estimated_robot_pos[2]):
        pt = np.floor(np.divide(estimated_robot_pos[0], scale)).astype(int)
        frame = cv2.circle(frame, (pt[0], pt[1]), radius=5, color=(255, 0, 255), thickness=-1)
        pt2 = pt.copy()
        LEN = 20
        pt2[0] = int(pt[0] + scale*LEN*np.cos(estimated_robot_pos[1]))
        pt2[1] = int(pt[1] + scale*LEN*np.sin(estimated_robot_pos[1]))
        frame = cv2.line(frame, (pt[0], pt[1]), (pt2[0], pt2[1]), color=(255, 0, 255), thickness=3)
        
    font = cv2.FONT_HERSHEY_SIMPLEX 
    cv2.putText(frame, text, (10, 50), font, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    
    
        
    return frame





class Observer():
    
    def __init__(self, cap):
        self.cap = cap
        self.scale = 1.0;
        self.robot_pos = [np.array([0, 0]), 0, False, 0]
        self.obstacles_dilated = []
        self.obstacles = []
        self.targets = []
        self.frame = np.zeros((200, 200, 3), dtype=np.uint8)
        self.error_log = []
        self.start_time = time.time()
        self.text = ""
        
        
    def stabilize(self, cycles):
        #read a few frames for the camera to adjust
        try:
            for i in range(cycles):
                self.cap.read()
                time.sleep(0.1)
        except:
            self.add_error("cam error")
                    
            
    def capture(self):
        try:
            ret, self.frame = self.cap.read()
        except:
            self.add_error("cam error")
        
    def set_frame(self, frame):
        self.frame = frame.copy()
            
    def find_scale(self):
        if(not self.robot_pos[2]):
            self.add_error("robot not found, using scale=1")
            self.scale = 1
        else:   
            self.scale = ROBOT_LEN/self.robot_pos[3]
        return self.scale
    
    def find_obstacles(self):
        self.obstacles_dilated, self.obstacles, ret = detect_obstacles(self.frame, self.scale)
        return self.obstacles_dilated
    
    def find_targets(self):
        self.targets, ret = detect_targets(self.frame, self.scale)
        return self.targets
    
    def find_robot(self):
        self.robot_pos, ret = detect_robot(self.frame, self.scale)
        if(not self.robot_pos[2]):
            self.add_error("robot not found")
        return self.robot_pos
    
    def get_robot_pos(self):
        return self.robot_pos

    def get_obstacles(self):
        return self.obstacles_dilated
    
    def get_obstacles_original(self):
        return self.obstacles
    
    def get_targets(self):
        return self.targets
    
    def get_scale(self):
        return self.scale
    
    def set_text(self, text):
        self.text = text;
    
    def debug_output(self, trajectory, estimated_robot_pos=[np.array([0, 0]), 0, False, 0]):
        frame = debug_output(self.frame, self.robot_pos, self.targets, self.obstacles, trajectory, estimated_robot_pos, self.text, self.scale)
        return frame
    
    def add_error(self, error):
        self.error_log.append([time.time()-self.start_time, error])

    def get_error_log(self):
        return self.error_log
        
    
    
    






