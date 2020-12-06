import sys
import time
import cv2
import serial
import math
from threading import Timer
import numpy as np
sys.path.append("localNavigation")
from Thymio import Thymio

############# CONSTANTES ######################
# speed factors
SENSOR_SCALE = 1500
MEMORY_FACTOR = 10
BASE_SPEED_HIGH = 150
BASE_SPEED_LOW = 75

# PID controler
KP = 100
KI = 3.5
KD = 8
ERROR_SATUDATION = 10

# tolerance for unprecision
TOLERENCE_POSITION = 10


############# GLOBAL VARIABLES ######################
value_proximity=[0,0,0,0,0,0,0]
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



############# FUNCTION DEFINITIONS ######################

class RepeatedTimer(object):
    """
    definition of timer, which can be called in a fixed interval
    """
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
    """
    This function should be called in order to connect Thymio
    "Thymio is connected :)" will be sent once the connexion is successful
    """
    global th
    th = Thymio.serial(port="COM5", refreshing_rate=0.1)
    time.sleep(10) # To make sure the Thymio has had time to connect
    print("Thymio is connected :)")


def deconnexion_thymio():
    """
    Deconnect Thymio once mission is accomplished
    """
    time.sleep(1)   # wait til all datas are transmised
    th.close()


def measure_sensor():
    """
    read the measurements of Thymio. The measurements 
    datas will be stored in global variables
    """
    global value_proximity
    global value_speed

    value_proximity=th['prox.horizontal']
    value_speed=[th['motor.left.speed'],th['motor.right.speed']]
    for i in range(2):
        if value_speed[i]>600:
            value_speed[i]=value_speed[i]-2**16

    return value_proximity,value_speed


def get_sensor_value():
    """
    The accessor function of measurements from this module.
    return: measurements stored in global variables
    """
    return value_proximity,value_speed



def get_position(robot_pos):
    """
    This function gets the variable containing the information of position 
    and angle of this robot, unpack this variable to extract and to store 
    these information into the global variables
    
    param: 
    robot_pos :containing position of robot, angle of robot, bool whether robot is detected
    
    return: 
    actual_position,actual_angle : actual position and angle of robot
    """
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
    """
    This function computes the error between the actual 
    angle and the target angle, we will try to eliminate
    this error with PID controler 
    
    params: 
    actual_position,goal,actual_angle
    
    return: 
    error
    """
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



def follow_the_way_to_dream(actual_position,goal,actual_angle):
    '''
    This function aims to command the motors of Thymio based on
    the angle error previously calculated.
    
    It has 2 parts of controls to the motor: 
        - speed for tracking the way to goal if no obstacle is detected
        - speed of local avoidance if the obstacle is detected
    
    params: 
    actual_position,goal,actual_angle
    '''
    global error_sum 
    global value_proximity
    global speed_avoidance_l_prev
    global speed_avoidance_r_prev

    

    if no_detection == False:

        '''
        part 1: local avoidance motor speed control
        Thymio will turn in order to avoid the obstacle
        base on the proximity sensor, it uses the memory 
        in order to "memorize" the existance of obstacle. 
        So Thymio can turn more to avoid it.
        
        '''
        x = np.array([0,0,0,0,0,0,0,0,0])      # array containing the measurement datas and memorized speeds
        
        
        # ponderation of importance of each sensor contributing the rotation [1:7] 
        # amplitude of movement for each motor due to avoidance [8:9]
        w_l = np.array([40,  20, -20, -20, -40,  30, -10, 8, 0])
        w_r = np.array([-40, -20, 20,  20,  40, -10,  30, 0, 8])

        x[:7]= np.array(value_proximity) / SENSOR_SCALE     # compute the roration due to obstacle
        x[7] = speed_avoidance_l_prev / MEMORY_FACTOR       # memory degradation
        x[8] = speed_avoidance_r_prev / MEMORY_FACTOR 

        speed_avoidance_l = np.sum(x * w_l)
        speed_avoidance_r = np.sum(x * w_r)
        speed_avoidance_l_prev=speed_avoidance_l
        speed_avoidance_r_prev=speed_avoidance_r


        #if memory to abstacle avoidance mode is still existing, higher basis speed to pass through
        if x[7] != 0 or x[8] != 0:
            base_speed = BASE_SPEED_HIGH
        else : 
            base_speed = BASE_SPEED_LOW

        '''
        part 2: PID motor speed control
        Thymio will try to ajuste its move orientation and turn in order to aligne 
        to the goal all the time
        
        '''

        error = calculate_error(actual_position,goal,actual_angle)
        

        if error_sum > ERROR_SATUDATION:
            error_sum = ERROR_SATUDATION
        if error_sum < -ERROR_SATUDATION:
            error_sum = -ERROR_SATUDATION
        
        
        # Compute the speed relative to PID controler
        vitesse_PID = KP * error + KI * error_sum + KD *(error-error_prev)

        # combining the final speed 
        speed_l = base_speed + vitesse_PID + speed_avoidance_l
        speed_r = base_speed - vitesse_PID + speed_avoidance_r

        move(int(speed_l),int(speed_r))
  
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
    """
    param: actual_position,goal_actual
    return: True or False depending on if Thymio is on the goal   
    """ 
    if (abs(goal_actual[0] - actual_position[0]) <= TOLERENCE_POSITION) and (abs(goal_actual[1] - actual_position[1]) <= TOLERENCE_POSITION) :   
        return True
    else:
        return False


def mission_accomplished():
    """
    param: none
    This function prints "Mission is accomplished!! " when Thymio has passed all the points  
    """ 
    stop()
    print("Mission is accomplished!! ")



