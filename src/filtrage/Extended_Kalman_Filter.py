import numpy as np
import matplotlib.pyplot as plt
from sympy import Symbol, symbols, Matrix, sin, cos

# DEFINE
ROBOT_DIAMETER = 95 #[mm]
PX = 0
PY = 1
PHI = 2
V = 3
PHI_DOT = 4

ALMOST_ZERO = 0.0000001
ALMOST_ALMOST_ZERO = 0.0000001
CONVERSION_FACTOR = 0.32

IPOS = 0
IANGLE = 1
IVISIBLE = 2
ILENGTH = 3

IX = 0
IY = 1

IRIGHT = 0
ILEFT = 1



class Kalman():

    def __init__(self, robot_pos):  
       
        # Initailise Variables
        numstates = 5 # number of states (px, py, phi, v, phi_dot)
        self.camera_avilable = False
        dt = 1.0/10.0 # Initial sample rate is set to 10Hz
        self.P = 0
        self.Q = 0
        self.R = 0
        self.I = 0
        self.x = 0
        self.measurements = 0


        # Initial Uncertainty matrix P0
        initial_uncertanty = 1000.0
        self.P = np.diag([initial_uncertanty, initial_uncertanty, initial_uncertanty, initial_uncertanty, initial_uncertanty])
        #print('P = ')
        #print(self.P, self.P.shape)

        # Process Noise Covariance Matrix Q0
        sCamera = 5  # Tuned parameter related to the camera
        sCourse  = 0.087222222 # Tuned parameter related to the camera
        sSpeed= 6.15 # Tuned parameter related to the robot
        sYawrate = 0.1 # Tuned parameter related to the robot
        self.Q = np.diag([sCamera, sCamera, sCourse, sSpeed, sYawrate])
        #print('Q = ')
        #print(self.Q, self.Q.shape)

        # Measurement Noise Covariance R
        varCamera = 4 # Tuned parameter related to the camera
        varrot = 0.05 # Tuned parameter related to the camera
        varspeed = 6.15 # Tuned parameter related to the robot
        varyaw = 0.1 # Tuned parameter related to the robot
        self.R = np.diag([varCamera, varCamera, varrot, varspeed, varyaw])
        #print('R =')
        #print(self.R, self.R.shape)

        # Identity Matrix I
        self.I = np.eye(numstates)
        #print('I =')
        #print(self.I, self.I.shape)


        # Initial State Vector
        # The states are (px, py, fi, v, w) = ([mm], [mm], [rad],[mm/s],[rad/s])
        self.x = np.matrix([[int(robot_pos[IPOS][IX]), int(robot_pos[IPOS][IY]), float(robot_pos[IANGLE]), 0.0, 0.0]]).T
        #print('x =')
        #print(self.x, self.x.shape)


        # Initial measurement vector  (px, py, fi, v, w) = ([mm], [mm], [rad],[mm/s],[rad/s])  Will be mesured
        self.measurements = np.matrix([[int(robot_pos[IPOS][IX]), int(robot_pos[IPOS][IY]), float(robot_pos[IANGLE]), 0.0, 0.0]]).T
        #print('measurements = ')
        #print(self.measurements)

    def estimate(self, dt):
        
        # Copy variabel for better understanding of formulas
        P = self.P  
        Q = self.Q 
        R = self.R 
        I = self.I 
        x = self.x 


        #  (Prediction) Prediction the state ahead
        #-------------------------------------------------------------------------------------

        # Attacking problem of driving "almost" straight                                      
        if np.abs(self.measurements[PHI_DOT])<ALMOST_ZERO: # Driving straight
            x[PX] = x[PX] + x[V]*dt * np.cos(x[PHI])
            x[PY] = x[PY] + x[V]*dt * np.sin(x[PHI])
            x[PHI] = x[PHI]
            x[V] = x[V]
            x[PHI_DOT] = ALMOST_ALMOST_ZERO # avoid numerical issues in Jacobians
        else: # otherwise
            x[PX] = x[PX] + (x[V]/x[PHI_DOT]) * (np.sin(x[PHI_DOT]*dt+x[PHI]) - np.sin(x[PHI]))
            x[PY] = x[PY] + (x[V]/x[PHI_DOT]) * (-np.cos(x[PHI_DOT]*dt+x[PHI])+ np.cos(x[PHI]))
            x[PHI] = (x[PHI] + x[PHI_DOT]*dt + np.pi) % (2.0*np.pi) - np.pi
            x[V] = x[V]
            x[PHI_DOT] = x[PHI_DOT]
        #print('x = ')
        #print(x)

        # Calculation of the Jacobian of our dynamic matrix A
        a13 = float((x[V]/x[PHI_DOT]) * (np.cos(x[PHI_DOT]*dt+x[PHI]) - np.cos(x[PHI])))
        a14 = float((1.0/x[PHI_DOT]) * (np.sin(x[PHI_DOT]*dt+x[PHI]) - np.sin(x[PHI])))
        a15 = float((dt*x[V]/x[PHI_DOT])*np.cos(x[PHI_DOT]*dt+x[PHI]) - (x[V]/x[PHI_DOT]**2)*(np.sin(x[PHI_DOT]*dt+x[PHI]) - np.sin(x[PHI])))
        a23 = float((x[V]/x[PHI_DOT]) * (np.sin(x[PHI_DOT]*dt+x[PHI]) - np.sin(x[PHI])))
        a24 = float((1.0/x[PHI_DOT]) * (-np.cos(x[PHI_DOT]*dt+x[PHI]) + np.cos(x[PHI])))
        a25 = float((dt*x[V]/x[PHI_DOT])*np.sin(x[PHI_DOT]*dt+x[PHI]) - (x[V]/x[PHI_DOT]**2)*(-np.cos(x[PHI_DOT]*dt+x[PHI]) + np.cos(x[PHI])))
        JA = np.matrix([[1.0, 0.0, a13, a14, a15],
                        [0.0, 1.0, a23, a24, a25],
                        [0.0, 0.0, 1.0, 0.0, dt],
                        [0.0, 0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 1.0]])


        # Projection of the error covariance ahead
        P = JA*P*JA.T + Q

        # (Correction) Measurement Update 
        # ---------------------------------------------------------------------------
    
        # CHecking if the camera measurements are avilable, form that we addapt the jacobian H
        if self.camera_avilable: 
            JH = np.diag([1.0, 1.0, 1.0, 1.0, 1.0])
        else: # every other step
            JH = np.diag([0.0, 0.0, 0.0, 1.0, 1.0])  

        # Calculating the Kalman gain     
        S = JH*P*JH.T + R
        K = (P*JH.T) * np.linalg.inv(S)

        # Updating the estimate 
        temp = self.measurements - (JH*x)                         
        x = x + (K*temp)

        # Updating the error covariance P
        P = (I - (K*JH))*P

        # Past again the copied value which were changed
        self.P = P  
        self.I = I 
        self.x = x 

        # If there is no error, output True (always true)
        self.camera_avilable = True

        # Return the state vector in the correct form, so that it can be used correctly
        output = [np.array([x.item(0), x.item(1)]), x.item(2), self.camera_avilable]  
        return output

    def update_measurements(self, camera_data, thymio_data):

        # Extract the information
        px = camera_data[IPOS][IX]
        py = camera_data[IPOS][IY]
        phi = camera_data[IANGLE]
        left_speed = CONVERSION_FACTOR*thymio_data[ILEFT]
        right_speed = CONVERSION_FACTOR*thymio_data[IRIGHT]

        # Converting the left and right speed, to an average speed and yawrate
        speed = (left_speed + right_speed)/2
        yawrate = (right_speed - left_speed)/(ROBOT_DIAMETER) 

        # Store the information at the right place
        self.camera_avilable = camera_data[IVISIBLE]
        self.measurements = np.matrix([px, py, phi, speed, yawrate]).T

    


