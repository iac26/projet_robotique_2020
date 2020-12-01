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
TOLERENCE_POSITION = 10
ERROR_TOLERENCE=0.1
THRESHOLD_DIST = 2000

############# GLOBAL VARIABLES ######################
counter = 0
value_proximity=[0,0,0,0,0,0,0]
value_acceleration=[0,0]
value_speed=[0,0]
actual_position=[0,0]
actual_angle=0
actual_goal=[0,0]
no_detection=False
error_sum = 0
error = 0
error_prev = 0
speed_avoidance_l_prev=0
speed_avoidance_r_prev=0




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
    th = Thymio.serial(port="COM5", refreshing_rate=0.1)
    time.sleep(10) # To make sure the Thymio has had time to connect
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
    
    return value_proximity,value_acceleration,value_speed

def get_sensor_value():
    return value_proximity,value_acceleration,value_speed



def get_position(robot_pos):
    global actual_angle
    global actual_position
    global no_detection
  
    if robot_pos[2]==True:
        no_detection= False
        #print("robot position:",robot_pos[0])
        #print("robot angle:",robot_pos)
        actual_position= [robot_pos[0][0],robot_pos[0][1]] 

        actual_angle=robot_pos[1]
        return actual_position,actual_angle
    else: 
        no_detection=True
        return actual_position,actual_angle


def calculate_error(actual_position,goal,actual_angle):
    global error_sum 
    global no_detection
    global error
    global error_prev
    goal_array = np.array([goal[0],goal[1]])
    actual_position_array = np.array([actual_position[0],actual_position[1]])
   
    direction = goal_array - actual_position_array
    angle = np.arctan2(direction[1], direction[0])
    error_prev = error
    error = -actual_angle + angle 

    if error < -np.pi:
        error += 2*np.pi 
    if error >  np.pi:
        error -= 2*np.pi 

    error_sum += error

    return error



""" 
   if no_detection==False:
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
        print("actual pos: ",actual_position)
        print("actual goal: ",goal)
    else:
        error=0
"""


def follow_the_way_to_dream(actual_position,goal,actual_angle):
    """
    base_speed, kp,ki à tunner
    """
    global error_sum 
    global value_proximity
    global counter
    global speed_avoidance_l_prev
    global speed_avoidance_r_prev

    if no_detection==False:
        x=np.array([0,0,0,0,0,0,0,0,0])
        sensor_scale = 1500
        w_l = np.array([40,  20, -20, -20, -40,  30, -10, 8, 0])
        w_r = np.array([-40, -20, 20,  20,  40, -10,  30, 0, 8])
        x[:7]= np.array(value_proximity) / sensor_scale
        x[7] = speed_avoidance_l_prev / 10 #10
        x[8] = speed_avoidance_r_prev / 10 #10

        speed_avoidance_l = np.sum(x * w_l)
        speed_avoidance_r = np.sum(x * w_r)
        speed_avoidance_l_prev=speed_avoidance_l
        speed_avoidance_r_prev=speed_avoidance_r

        if x[7] != 0 or x[8] != 0:
            base_speed = 150
        else : 
            base_speed = 75 


        error = calculate_error(actual_position,goal,actual_angle)
        print("error: ",error)
        """
        if any(np.array(value_proximity) > 3000) :
            #counter=20

        if counter > 0:
            counter -= 1
            print("counter:",counter)
            kp = 5
            ki = 0
            base_speed = 100 #75


        else:
            kp = 100
            ki = 5
            base_speed = 50
        """

        
        kp = 100
        ki = 3.5
        kd = 8
 
        error_sat = 10

        if error_sum>error_sat:
            error_sum=error_sat
        if error_sum<-error_sat:
            error_sum=-error_sat
        
        print("error_sum: ",error_sum)

        vitesse_PID = kp*error+ki*error_sum + kd *(error-error_prev)

        speed_l = base_speed + vitesse_PID + speed_avoidance_l
        speed_r = base_speed - vitesse_PID + speed_avoidance_r
        

        print("speed left: ",speed_l)
        print("speed right: ",speed_r)

        move(int(speed_l),int(speed_r))

        """
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
            speed_final_l=w_obstacle[0]*speed_l+w_obstacle[1]*speed_l_a
            speed_final_r=w_obstacle[0]*speed_r+w_obstacle[1]*speed_r_a
            print("error: ", error)
            #move(0,0)
            move(int(speed_final_l),int(speed_final_r))
            
        else : 
            print("error: ", error)
            #move(0,0)
            move(int(speed_l),int(speed_r))
        """    
    else:
        stop()
 
    

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


"""
def local_avoidance (actual_position,goal,actual_angle):
    if no_detection==False:
        calculate_error(actual_position,goal,actual_angle)
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
            speed_final_l=w_obstacle[0]*speed_l+w_obstacle[1]*speed_l_a
            speed_final_r=w_obstacle[0]*speed_r+w_obstacle[1]*speed_r_a
            print("error: ", error)
            #move(0,0)
            move(int(speed_final_l),int(speed_final_r))
            
        else : 
            print("error: ", error)
            #move(0,0)
            move(int(speed_l),int(speed_r))
    else:
        stop()



def detect_obstacles (): # upload if obstacle is present
    global obstacles_bool
    obstacles_bool = 0
    for x in value_proximity[0:5]:  #front horiton prox
        if x>=THRESHOLD_DIST:     # 3000 --> 7cm white surface
            obstacles_bool = 1


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
"""

