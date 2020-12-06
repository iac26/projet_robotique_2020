import cv2
import numpy as np
import matplotlib.pyplot as plt
import time



RED = (0, 0, 255)
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)
LIGHTBLUE = (255, 127, 0)
TURQUOISE = (255, 255, 0)
PINK = (255, 0, 255)
ORANGE = (0, 127, 255)
YELLOW = (0, 255, 255)
WHITE = (255)

RED_LOW  = [150, 100, 100]
RED_HIGH = [179, 255, 255]

GREEN_LOW  = [41, 64, 0]
GREEN_HIGH = [85, 140, 140]

BLUE_LOW  = [87, 129, 80]
BLUE_HIGH = [131, 255, 255]


IX=0
IY=1

H_MIN = 0
S_MIN = 0
V_MIN = 0
H_MAX = 179
S_MAX = 255
V_MAX = 255


AREA_THRESH = 100
MERGE_THRESH = 0.04
EPSILON = 40

LP_MODE_REL = 0
LP_MODE_ABS = 1

ROBOT_LEN = 100

DIL_COEFF = 100
EXP_RATIO = 60

DIL_COEFF_K = 8
NB_ITER = 10

NB_VERTEX_TRIANGLE = 3
TRIANGLE_LONG_SHORT_RATIO=2

FIRST = 0
SECOND = 1
THIRD = 2

IPOS = 0
IANGLE = 1
IVISIBLE = 2
ILENGTH = 3



def cleanup_contours(contours, mode=0):
    """
    This function filters contours by removing small ones and reducind the number of verices.
    contours    opencv standard list of contours
    mode        Vertices merge method (0=relative to arclength, 1=fixed)

    returns     Filtered and cleaned list of contours
    """
    
    clean_contours = []
    
    for cnt in contours:
        # only take big enough contours
        if (cv2.contourArea(cnt) >= AREA_THRESH):
            if mode == LP_MODE_REL:
                epsilon = MERGE_THRESH*cv2.arcLength(cnt,True)
            else:
                epsilon = EPSILON
            approx = cv2.approxPolyDP(cnt,epsilon,True)
            clean_contours.append(approx)
            
    return clean_contours


def find_color(frame, hsv_low, hsv_high):
    """
    this function finds the contours of colored areas in a frame
    frame       standard opencv BGR image
    hsv_low     hsv color lower boundary (np array)
    hsv_high    hsv color upper boundary (np array)

    returns     Filtered and cleaned list of contours
    """

    #transform frame to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #this gives us a mask of the colored sections
    mask = cv2.inRange(hsv,  hsv_low, hsv_high)
    
    #find contours
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL  , cv2.CHAIN_APPROX_SIMPLE)
    
    return cleanup_contours(contours)
    



def detect_robot(frame, scale=1):
    """
    this function finds the robot in the frame. it also finds it's position, size and orientation.
    frame       standard opencv BGR image
    scale       scale of the world
    
    returns     an array [ndarray(x, y), angle, visible T/F, size in px], a frame with debug information overlay
    """
    blue_low = np.array(BLUE_LOW, np.uint8)
    blue_high = np.array(BLUE_HIGH, np.uint8)
    frame = frame.copy()
    
    clean_contours = find_color(frame, blue_low, blue_high)
    
    good_cnt = []
    
    for cnt in clean_contours:
        if(len(cnt) == NB_VERTEX_TRIANGLE):
            K = TRIANGLE_LONG_SHORT_RATIO
            A = 0
            B = 0
            C = 0
            dAB = 0
            dBC = 0
            dCA = 0
            p1 = cnt[FIRST][0]
            p2 = cnt[SECOND][0]
            p3 = cnt[THIRD][0]
            d1 = np.linalg.norm(p2-p1)
            d2 = np.linalg.norm(p3-p2)
            d3 = np.linalg.norm(p1-p3)
            min_ix = np.argmin([d1, d2, d3])
            if(min_ix == FIRST):
                A = p3
                B = p2
                C = p1
                dAB = d2
                dBC = d1
                dCA = d3
            elif(min_ix == SECOND):
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
            score = abs(dAB-dCA)+abs(K*dBC - dAB)+abs(K*dBC - dCA)/np.linalg.norm(dAB)
            good_cnt.append([A, B, C, score])
                       
    good_cnt = sorted(good_cnt, key = lambda x: x[3])
    
    robot_pos = [np.array([0, 0]), 0, False, 0]
    
    if(len(good_cnt) > 0):
        robot_visible = True
        A = good_cnt[0][FIRST]
        B = good_cnt[0][SECOND]
        C = good_cnt[0][THIRD]
        D = (np.mean([[B, C]], axis=IY))[0]
        
        
        Center = (np.mean([[A, B, C]], axis=IY))[0]

        direction = A - D
        
        size = np.linalg.norm(direction)
        
        angle = np.arctan2(direction[IY], direction[IX])
        
        frame = cv2.line(frame, (int(D[IX]), int(D[IY])), (int(A[IX]), int(A[IY])), color=RED, thickness=1)
        frame = cv2.circle(frame, (int(Center[IX]), int(Center[IY])), radius=5, color=RED, thickness=-1)
        Center = np.multiply(Center, scale).astype(int)
        text =  "position: ({:0.2f}, {:0.2f}) angle: {:0.4f}".format(Center[IX], Center[IY], angle)
        font = cv2.FONT_HERSHEY_SIMPLEX 
        cv2.putText(frame, text, (10, 50), font, 0.5, GREEN, 1, cv2.LINE_AA)
        
        
        robot_pos = [Center, angle, True, size]
        
    return robot_pos, frame


def detect_obstacles_alt(frame, scale=1):
    """
    this function finds the obstacles and their dilation. we use geometric calculations to dilate the obstacles.   
    frame       standard opencv BGR image
    scale       scale of the world

    returns     dilated_contours, original_contours, frame with debug overlay
    """

    frame = frame.copy()
    red_low = np.array(RED_LOW, np.uint8)
    red_high = np.array(RED_HIGH, np.uint8)
    
    clean_contours = find_color(frame, red_low, red_high)
            
    original_contours = []
    dil_contour = []
    for cnt in clean_contours:
        mom = cv2.moments(cnt)
        if mom["m00"] != 0:
            cx = int(mom["m10"] / mom["m00"])
            cy = int(mom["m01"] / mom["m00"])
            C = np.array([cx, cy])
        else:
            C = np.array([0, 0])
        ncnt = []
        ocnt = []
        cnt = list(cnt)
        cnt.append(cnt[0])
        print(cnt)
        for i, _ in enumerate(cnt[0:-1]):
            pt1 = cnt[i][0]
            pt2 = cnt[i+1][0]
            seg = pt2-pt1
            d = seg/np.linalg.norm(seg)
            n = np.array([-seg[IY], seg[IX]])/np.linalg.norm(seg)

            N = pt1-C
            N = N/np.linalg.norm(N)
            npt = (pt1+(DIL_COEFF+EXP_RATIO/2)/scale*N).astype(int)

            npt1 = (pt1+DIL_COEFF/scale*n - EXP_RATIO/scale*d).astype(int)
            npt2 = (pt2+DIL_COEFF/scale*n + EXP_RATIO/scale*d).astype(int)

            frame = cv2.circle(frame, (npt[IX], npt[IY]), radius=5, color=YELLOW, thickness=-1)
            frame = cv2.circle(frame, (npt1[IX], npt1[IY]), radius=5, color=ORANGE, thickness=-1)
            frame = cv2.circle(frame, (npt2[IX], npt2[IY]), radius=5, color=RED, thickness=-1)

            ncnt.append(npt)
            ncnt.append(npt1)
            ncnt.append(npt2)
            ocnt.append(cnt[i][0])
        
        dil_contour.append(np.array(ncnt))
        original_contours.append(np.multiply(ocnt, scale).astype(int))
        
        
    
    cv2.drawContours(frame, clean_contours, -1, GREEN, 3)
    
    black = np.zeros(frame.shape[:2], dtype=np.uint8)
    
    for i in range(len(dil_contour)):
        cv2.drawContours(black, dil_contour, i, WHITE, -1)

    #plt.imshow(frame)
    
    #find contours
    
    contours, hierarchy = cv2.findContours(black, cv2.RETR_EXTERNAL  , cv2.CHAIN_APPROX_SIMPLE)
    
    clean_dil_contours = cleanup_contours(contours, 1)

    scaled_contours = []
    for cnt in clean_dil_contours:
        ncnt = []
        for pt in cnt:
            frame = cv2.circle(frame, (pt[0][IX], pt[0][IY]), radius=5, color=BLUE, thickness=-1)
            ncnt.append(pt[0])
        scaled_contours.append(np.multiply(ncnt, scale).astype(int))
    
    
    return scaled_contours, original_contours, frame


def detect_obstacles(frame, scale=1):
    """
    this function finds the obstacles and their dilation. We use openCV to dilate the obstacles  
    frame       standard opencv BGR image
    scale       scale of the world

    returns     dilated_contours, original_contours, frame with debug overlay
    """

    frame = frame.copy()
    red_low = np.array(RED_LOW, np.uint8)
    red_high = np.array(RED_HIGH, np.uint8)
    
    clean_contours = find_color(frame, red_low, red_high)

    cv2.drawContours(frame, clean_contours, -1, GREEN, 3)
            
    original_contours = []
    for cnt in clean_contours:
        ocnt = []
        for pt in cnt:
            ocnt.append(pt[0])
        original_contours.append(np.multiply(ocnt, scale).astype(int))
        
    

    ## DILATATION
    black = np.zeros(frame.shape[:2], dtype=np.uint8)
    

    for i in range(len(clean_contours)):
        cv2.drawContours(black, clean_contours, i, WHITE, -1)

    #plt.imshow(black)
    
    # dilatation
    kernSize = 2*int(DIL_COEFF_K/scale)+1
    kernel = np.ones((kernSize,kernSize),np.uint8)
    black = cv2.dilate(black, kernel, iterations = NB_ITER)

    plt.figure()

    #plt.imshow(black)
    
    
    contours, hierarchy = cv2.findContours(black, cv2.RETR_EXTERNAL  , cv2.CHAIN_APPROX_SIMPLE)
    
    clean_dil_contours = cleanup_contours(contours)

    scaled_contours = []
    for cnt in clean_dil_contours:
        ncnt = []
        for pt in cnt:
            frame = cv2.circle(frame, (pt[0][IX], pt[0][IY]), radius=5, color=BLUE, thickness=-1)
            ncnt.append(pt[0])

        scaled_contours.append(np.multiply(ncnt, scale).astype(int))
    
    
    return scaled_contours, original_contours, frame


def detect_targets(frame, scale=1):
    """
    this function finds the targets 
    frame       standard opencv BGR image
    scale       scale of the world

    returns     list of centroids of the targets, frame with debug overlay
    """
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
            
        
            
    cv2.drawContours(frame, clean_contours, -1, GREEN, 3)
    scaled_centroids = []
    
    for pt in centroids:
        frame = cv2.circle(frame, (pt[IX], pt[IY]), radius=5, color=BLUE, thickness=-1)
        scaled_centroids.append(np.multiply(pt, scale).astype(int).tolist())
        
    return scaled_centroids, frame




def detect_scale(frame):
    """
    This function calculates the scale of the world from the size of the robot
    """
    robot_pos, ret = detect_robot(frame)
    if robot_pos[IVISIBLE]:
        return ROBOT_LEN/robot_pos[ILENGTH]
    else:
        return 1



def debug_output(frame, robot_pos, targets, obstacles, trajectory, estimated_robot_pos, text, scale):
    """
    This function provides a debug overlay on a frame.
    frame       standard openCV frame
    robot_pos   robot pos array (output of find_robot)
    targets     list of targets
    obstacles   list of contours
    trajectory  list of points
    estimated_robot_pos robot pos array (output of kalman filter)
    text        some random text to display
    scale       the scale of the world

    returns a frame with debug overlay
    """
    frame = frame.copy()
    for pt in targets:
        pt = np.floor(np.divide(pt, scale)).astype(int)
        frame = cv2.circle(frame, (pt[IX], pt[IY]), radius=5, color=GREEN, thickness=-1)
        
    for cnt in obstacles:
        for pt in cnt:
            pt = np.floor(np.divide(pt, scale)).astype(int)
            frame = cv2.circle(frame, (pt[IX], pt[IY]), radius=5, color=RED, thickness=-1)
    
    fst = 0
    
    trajectory = np.divide(trajectory, scale).astype(int)
    for pt in trajectory:
        if fst == 0:
            fst = 1
            lpt = pt  
        else:
            frame = cv2.line(frame, (int(lpt[IX]), int(lpt[IY])), (int(pt[IX]), int(pt[IY])), color=PINK, thickness=3) 
        lpt = pt

    if(robot_pos[IVISIBLE]):
        pt = np.floor(np.divide(robot_pos[IPOS], scale)).astype(int)
        frame = cv2.circle(frame, (pt[IX], pt[IY]), radius=5, color=BLUE, thickness=-1)
        pt2 = pt.copy()
        LEN = 20
        pt2[IX] = int(pt[IX] + scale*LEN*np.cos(robot_pos[IANGLE]))
        pt2[IY] = int(pt[IY] + scale*LEN*np.sin(robot_pos[IANGLE]))
        frame = cv2.line(frame, (pt[IX], pt[IY]), (pt2[IX], pt2[IY]), color=BLUE, thickness=3)
    
    if(estimated_robot_pos[IVISIBLE]):
        pt = np.floor(np.divide(estimated_robot_pos[IPOS], scale)).astype(int)
        frame = cv2.circle(frame, (pt[IX], pt[IY]), radius=5, color=GREEN, thickness=-1)
        pt2 = pt.copy()
        LEN = 20
        pt2[IX] = int(pt[IX] + scale*LEN*np.cos(estimated_robot_pos[IANGLE]))
        pt2[IY] = int(pt[IY] + scale*LEN*np.sin(estimated_robot_pos[IANGLE]))
        frame = cv2.line(frame, (pt[IX], pt[IY]), (pt2[IX], pt2[IY]), color=GREEN, thickness=3)
        
    font = cv2.FONT_HERSHEY_SIMPLEX 
    cv2.putText(frame, text, (10, 50), font, 0.5, GREEN, 1, cv2.LINE_AA)
    
    
        
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
            #print("robot not found, using scale=1")
        else:   
            self.scale = ROBOT_LEN/self.robot_pos[ILENGTH]
            #print(self.scale)
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
        self.text = text
    
    def debug_output(self, trajectory, estimated_robot_pos=[np.array([0, 0]), 0, False, 0]):
        frame = debug_output(self.frame, self.robot_pos, self.targets, self.obstacles_dilated, trajectory, estimated_robot_pos, self.text, self.scale)
        return frame
    
    def add_error(self, error):
        self.error_log.append([time.time()-self.start_time, error])

    def get_error_log(self):
        return self.error_log
        
    
    
    






