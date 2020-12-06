print("importing modules...")
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
print("OK")


#===== INITIALISATION =====

print("opening capture device...")
try:
    cap = cv2.VideoCapture(0)
    print("OK")
    pass
except:
    print("FAIL")
    cap = None
    


#initialize observer and stabilize camera for 20 frames
print("initializing capture device...")
observer = vision.Observer(cap)
observer.stabilize(20)
print("OK")

#capture one frame and use it to find:
# - robot
# - scale
# - obstacles
# - targets
print("capturing initial parameters...")
observer.capture()
observer.find_robot() #find robot for scale computation
scale = observer.find_scale()
obstacles = observer.find_obstacles()
targets = observer.find_targets()
robot_pos = observer.find_robot() #find robot with newly computed scale
while not robot_pos[2]:
    print("FAIL\nRobot not found...")
    observer.capture()
    observer.find_robot() #find robot for scale computation
    scale = observer.find_scale()
    robot_pos = observer.find_robot() #find robot with newly computed scale
    time.sleep(1)

print("OK")
print("world scale is: ", scale)
print("robot state is: ", robot_pos)
print("nb of obstacles is: ", len(obstacles))
print("nb of targets is: ", len(targets))


####
#plt.figure()
#plt.gca().invert_yaxis()
#globalNavigation.printGlobalNavigation(observer.get_obstacles_original(), obstacles, interestPoints = targets)


print("computing visibility graph...")
visibilityGraph, possibleDisplacement = globalNavigation.computeVisibilityGraph(obstacles)
targets.insert(0, [robot_pos[0][0], robot_pos[0][1]]) # the initial position of the Thymio is the starting point of the trajectory
print("OK")
#CHECK THAT NO POINTS ARE IN OBSTACLES
print("checking targets validity...")
pointsInObstacle = globalNavigation.InterestPointInObstacle(targets, visibilityGraph) 
while pointsInObstacle:
    print("FAIL\nOne of the interest point is in a dilated obstacle, please replace them...")
    targets.clear()
    observer.capture()
    targets = observer.find_targets()
    robot_pos = observer.find_robot()
    targets.insert(0, [robot_pos[0][0], robot_pos[0][1]])
    pointsInObstacle = globalNavigation.InterestPointInObstacle(targets, visibilityGraph)
    time.sleep(1)
print("OK")

print("computing trajectory...")
trajectory = globalNavigation.computeTrajectory(visibilityGraph, targets)
print("OK")


print("displaying initial information...")
final = observer.debug_output(trajectory)
print("OK")
print("close plot to continue...")
plt.imshow(cv2.cvtColor(final, cv2.COLOR_BGR2RGB))
plt.show()
print("OK")

#plt.figure()
#plt.gca().invert_yaxis()

#globalNavigation.printGlobalNavigation(observer.get_obstacles_original(), obstacles, interestPoints = targets, trajectory = trajectory)

print("initializing kalman...")
kalman = Extended_Kalman_Filter.Kalman(robot_pos)
print("OK")

print("connecting thymio...")
start_thymio.connexion_thymio()
print("OK")


count_trajectory=1
goal_actual=trajectory[count_trajectory]
nb_goal=len(trajectory)



last_time = time.time()
print("started mainloop (press q to quit)...")

while 1:
    time.sleep(0.1)
    
    value_speed = start_thymio.measure_sensor()
    
    observer.capture()
    
    robot_pos = observer.find_robot()
    
    dt = time.time()-last_time
    last_time = time.time()
    kalman.update_measurements(robot_pos, value_speed)
    estimated_robot_pos = kalman.estimate(dt)
    actual_position,actual_angle = start_thymio.get_position(estimated_robot_pos)
    
    
    if start_thymio.detect_trajectory(actual_position,goal_actual) and count_trajectory < nb_goal-1:         # upload goal 
        count_trajectory+=1
        goal_actual=trajectory[count_trajectory]
        #start_thymio.follow_the_way_to_dream(actual_position,goal_actual,actual_angle)
        

    elif start_thymio.detect_trajectory(actual_position,goal_actual) and count_trajectory == nb_goal-1:     # if all points are finished
        start_thymio.mission_accomplished()
        break

    #else:
     #   start_thymio.follow_the_way_to_dream(actual_position,goal_actual,actual_angle)    
      
    start_thymio.follow_the_way_to_dream(actual_position,goal_actual,actual_angle)
    
    final = observer.debug_output(trajectory, estimated_robot_pos)
    cv2.imshow('frame',final)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    

try:
    cap.release()
    
except:
    pass

start_thymio.stop()

time.sleep(0.5)

start_thymio.deconnexion_thymio()



cv2.destroyAllWindows()
    
    
    
    
    
    
    
    
    
    
    