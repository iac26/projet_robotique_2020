import sys
import time
import cv2
import serial
import math
from threading import Timer
import numpy as np
import matplotlib.pyplot as plt

sys.path.append("localNavigation")
from Thymio import Thymio

sys.path.append("vision")
import vision
############# CONSTANTES ######################
TOLERENCE_POSITION = 2
ERROR_TOLERENCE=0.1
THRESHOLD_DIST = 2000

############# GLOBAL VARIABLES ######################
obstacles_bool = 0
value_proximity=[0,0,0,0,0,0,0]
value_acceleration=[0,0]
value_speed=[0,0]
actual_position=[0,0]
actual_angle=0
actual_goal=[0,0]

error=0
error_sum=0




class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.interval   = interval
        self.function   = function
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False




def connexion_thymio():
    global th
    th = Thymio.serial(port="COM8", refreshing_rate=0.1)
    time.sleep(3) # To make sure the Thymio has had time to connect
    print("Thymio is connected :)")


def measure_sensor():
    global value_proximity
    global value_acceleration
    global value_speed

    value_proximity=th['prox.horizontal']
    value_acceleration=th['acc']
    value_speed=[th['motor.left.speed'],th['motor.right.speed']]
    for i in range(2):
        if value_speed[i]>600:
            value_speed[i]=value_speed[i]-2**16
    for i in range(3):
        if value_acceleration[i]>600:
            value_acceleration[i]=value_acceleration[i]-2**16
    
    #return value_proximity,value_acceleration,value_speed

def get_sensor_value():
    return value_proximity,value_acceleration,value_speed



def get_position(frame):
    global actual_angle
    global actual_position
    global robot_pos
    #get one frame
    
    scale = vision.detect_scale(frame)
    robot_pos, ret = vision.detect_robot(frame, scale)
    #get one frame

    obstacles, ret = vision.detect_obstacles(frame, scale)
    #print(obstacles)

    targets, ret = vision.detect_targets(frame, scale)
    #print(targets)
    #print(robot_pos)
    #print("robot position:",robot_pos[0])
    #print("robot angle:",robot_pos[1])
    if robot_pos[2]==True:
        #print("robot position:",robot_pos[0])
        #print("robot angle:",robot_pos)
        actual_position= [robot_pos[0][0],robot_pos[0][1]] 

        actual_angle=robot_pos[1]
        return actual_position,actual_angle
    else: 
        return actual_position,actual_angle
    
"""
def compute_angle_goal (actual_position,goal,actual_angle):
    return math.atan((goal[1]-actual_position[1])/(goal[0]-actual_position[0]))-actual_angle
"""

def compute_angle_goal (actual_position,goal):
    return math.atan((goal[1]-actual_position[1])/(goal[0]-actual_position[0]))

def calculate_error(goal):
    global error_sum 
    global error

    angle_b=compute_angle_goal(actual_position,goal)
    delta_x=goal[0]-actual_position[0]
    delta_y=goal[1]-actual_position[1]
    if delta_x >= 0 and delta_y <= 0:
        error=angle_b - actual_angle
    elif delta_x <= 0 and delta_y <= 0:
        error=-1*math.pi + angle_b - actual_angle
    elif delta_x < 0 and delta_y > 0:
        error=(math.pi) + angle_b - actual_angle 
    elif delta_x > 0 and delta_y > 0:
        error = angle_b - actual_angle 

    error_sum = error_sum + error




def follow_the_way_to_dream(actual_position,goal,actual_angle):
    """
    base_speed, kp,ki à tunner
    """
    global error_sum 
    global error
    base_speed = 25
    kp = 200
    ki = 5
    saturation = 500
    error_sat=500/ki
    angle_b=compute_angle_goal(actual_position,goal)
    delta_x=goal[0]-actual_position[0]
    delta_y=goal[1]-actual_position[1]

    if delta_x >= 0 and delta_y <= 0:
        error=angle_b - actual_angle
    elif delta_x <= 0 and delta_y <= 0:
        error=-1*math.pi + angle_b - actual_angle
    elif delta_x < 0 and delta_y > 0:
        error=(math.pi) + angle_b - actual_angle 
    elif delta_x > 0 and delta_y > 0:
        error = angle_b - actual_angle 


    error_sum = error_sum + error

    if error_sum>error_sat:
        error_sum=error_sat
    if error_sum<-error_sat:
        error_sum=-error_sat

    speed_l= base_speed + (kp*error+ki*error_sum)
    speed_r= base_speed - (kp*error+ki*error_sum)
 

    if speed_l>saturation:
        speed_l=saturation
    if speed_l<-saturation:
        speed_l=-saturation
    if speed_r>saturation:
        speed_r=saturation
    if speed_r<-saturation:
        speed_r=-saturation

    #print("total error: ",error_sum)    
    #print("error: ",error)    
    #print("speed left: ",speed_l)    
    #print("speed right: ",speed_r) 
    #move(int(speed_l),int(speed_r))
    return int(speed_l),int(speed_r)
    

def move(l_speed=500, r_speed=500, verbose=False):
    """
    Sets the motor speeds of the Thymio 
    param l_speed: left motor speed
    param r_speed: right motor speed
    param verbose: whether to print status messages or not
    """
    # Printing the speeds if requested
    if verbose: print("\t\t Setting speed : ", l_speed, r_speed)
    
    # Changing negative values to the expected ones with the bitwise complement
    l_speed = l_speed if l_speed>=0 else 2**16+l_speed
    r_speed = r_speed if r_speed>=0 else 2**16+r_speed

    # Setting the motor speeds
    th.set_var("motor.left.target", l_speed)
    th.set_var("motor.right.target", r_speed)

def stop(verbose=False):
    """
    param verbose: whether to print status messages or not
    """
    # Printing the speeds if requested
    if verbose:
        print("\t\t Stopping")

    # Setting the motor speeds
    th.set_var("motor.left.target", 0)
    th.set_var("motor.right.target", 0)

def detect_trajectory(actual_position,goal_actual): 
    if (abs(goal_actual[0]-actual_position[0])<=TOLERENCE_POSITION) and (abs(goal_actual[1]-actual_position[1])<=TOLERENCE_POSITION):   
        return True
    else:
        return False

def mission_accomplished():
    stop()
    print("Mission is accomplished!! ")






def local_avoidance (actual_position,goal,actual_angle):
    w_obstacle=[0.1,0.9]
    speed_base=30
    speed_a=100
    speed_l, speed_r = follow_the_way_to_dream(actual_position,goal,actual_angle)
    if obstacles_bool:
        direction = value_proximity.index(max(value_proximity[0:5]))
        if direction == 0 or direction == 1:
            speed_l_a= speed_a
            speed_r_a= -speed_a
        if direction == 3 or direction == 4:
            speed_l_a= -speed_a
            speed_r_a= speed_a
        if direction == 2 :
            speed_l_a= -speed_a
            speed_r_a= speed_a
        speed_final_l=speed_base+w_obstacle[0]*speed_l+w_obstacle[1]*speed_l_a
        speed_final_r=speed_base+w_obstacle[0]*speed_r+w_obstacle[1]*speed_r_a
        move(int(speed_final_l),int(speed_final_r))

            
        
    else : 
        speed_l_a= 0
        speed_r_a= 0
        speed_final_l=speed_l
        speed_final_r=speed_r
        move(int(speed_final_l),int(speed_final_r))




def detect_obstacles (): # upload if obstacle is present
    global obstacles_bool
    obstacles_bool = 0
    for x in value_proximity[0:5]:  #front horiton prox
        if x>=THRESHOLD_DIST:     # 3000 --> 7cm white surface
            obstacles_bool = 1

    #print(obstacles_bool)


def avoidance_move():
    global value_proximity
    base_speed = 25
    sensor_scale = 200
    w_l = np.array([40,  20, -20, -20, -40,  30, -10])
    w_r = np.array([-40, -20, -20,  20,  40, -10,  30])
    x = np.array(value_proximity) / sensor_scale
    #value_proximity
    speed_l= base_speed + np.sum(x * w_l)
    speed_r= base_speed + np.sum(x * w_r)
    move(int(speed_l),int(speed_r))


def normal_move():
    global error_sum
    global error
    base_speed = 25
    kp = 200
    ki = 5
    saturation = 500
    error_sat=500/ki

    if error_sum>error_sat:
        error_sum=error_sat
    if error_sum<-error_sat:
        error_sum=-error_sat

    speed_l= base_speed + (kp*error+ki*error_sum)
    speed_r= base_speed - (kp*error+ki*error_sum)
 

    if speed_l>saturation:
        speed_l=saturation
    if speed_l<-saturation:
        speed_l=-saturation
    if speed_r>saturation:
        speed_r=saturation
    if speed_r<-saturation:
        speed_r=-saturation

    #print("total error: ",error_sum)    
    #print("error: ",error)    
    #print("speed left: ",speed_l)    
    #print("speed right: ",speed_r) 
    move(int(speed_l),int(speed_r))
    #return int(speed_l),int(speed_r)


def change_mode(goal):
    detect_obstacles ()
    calculate_error(goal)

    if obstacles_bool==1:
        avoidance_move()

    if error<ERROR_TOLERENCE and obstacles_bool==0:
        normal_move()