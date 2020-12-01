import cv2
import numpy as np


def nothing(lol):
    pass

cap = cv2.VideoCapture(1)
     

cv2.namedWindow("Hsv Capture", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Hsv Capture", 600,600)

# create trackbars for color change
# IMPORTANT: You have to define the correct HSV opencv range hence 179,255,255
cv2.createTrackbar('H low', 'Hsv Capture', 0, 179, nothing)
cv2.createTrackbar('S low', 'Hsv Capture', 0, 255, nothing)
cv2.createTrackbar('V low', 'Hsv Capture', 0, 255, nothing)

cv2.createTrackbar('H high', 'Hsv Capture', 0, 179, nothing)
cv2.createTrackbar('S high', 'Hsv Capture', 0, 255, nothing)
cv2.createTrackbar('V high', 'Hsv Capture', 0, 255, nothing)

cv2.setTrackbarPos('H low', 'Hsv Capture', 83)
cv2.setTrackbarPos('S low', 'Hsv Capture', 165)
cv2.setTrackbarPos('V low', 'Hsv Capture', 100)

cv2.setTrackbarPos('H high', 'Hsv Capture', 120)
cv2.setTrackbarPos('S high', 'Hsv Capture', 255)
cv2.setTrackbarPos('V high', 'Hsv Capture', 255)

while(True):

    ret, imag = cap.read()
    #imag = cv2.imread("images/thymio_1.jpg")
    frame = cv2.cvtColor(imag, cv2.COLOR_BGR2HSV)
    
    

    # Trackbars realtime position
    h1 = cv2.getTrackbarPos('H low', 'Hsv Capture')
    s1 = cv2.getTrackbarPos('S low', 'Hsv Capture')
    v1 = cv2.getTrackbarPos('V low', 'Hsv Capture')

    h2 = cv2.getTrackbarPos('H high', 'Hsv Capture')
    s2 = cv2.getTrackbarPos('S high', 'Hsv Capture')
    v2 = cv2.getTrackbarPos('V high', 'Hsv Capture')

    #How to store the min and max values from the trackbars
    blue_MIN = np.array([h1, s1, v1], np.uint8)
    blue_MAX = np.array([h2, s2, v2], np.uint8)

    #After finding your values, you can replace them like this
    #blue_MIN = np.array([95, 123, 0], np.uint8)
    #blue_MAX = np.array([135, 207, 255], np.uint8)
            
    #Using inRange to find the desired range
    hsvCapture = cv2.inRange(frame,  blue_MIN, blue_MAX)
    
    contours, hierarchy = cv2.findContours(hsvCapture, cv2.RETR_EXTERNAL  , cv2.CHAIN_APPROX_SIMPLE)
    
    #clean contours
    
    AREA_THRESH = 200
    
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
            
            
    
    
    mask = hsvCapture
    mask_inv = cv2.bitwise_not(mask)
    
    blue = np.zeros(frame.shape, np.uint8)

    blue[:]=(255,0,0)
    
    bg = cv2.bitwise_and(imag,imag,mask = mask_inv)
    fg = cv2.bitwise_and(blue,blue,mask = mask)
    
    #final = cv2.add(bg,fg)
    final = imag
    
    cv2.drawContours(final, clean_contours, -1, (0,255,0), 3)
    
    #draw points
    
    
    for cnt in clean_contours:
        for pt in cnt:
            final = cv2.circle(final, (pt[0][0], pt[0][1]), radius=5, color=(0, 0, 255), thickness=-1)
            

    cv2.imshow('Hsv Capture', final)
    cv2.resizeWindow("Hsv Capture", 600,600)
    
    
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print(clean_contours)
        break

cap.release()
cv2.destroyAllWindows()