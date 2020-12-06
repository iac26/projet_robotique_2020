import numpy as np
import matplotlib.pyplot as plt
from sympy import Symbol, symbols, Matrix, sin, cos

# DEFINE
robot_diameter = 95 #[mm]
speed_conversion_factor = 0.349   # [thymio/s] --> [mm/s]


class Kalman():

    def __init__(self, robot_pos):  
       
        # Initailise Variables
        numstates = 5 # number of states (px, py, phi, v, phi_dot)
        self.camera_avilable = False
        dt = 1.0/10.0 # Initial sample rate is set to 50Hz
        self.P = 0
        self.Q = 0
        self.R = 0
        self.I = 0
        self.x = 0
        self.measurements = 0


        # Initial Uncertainty P0
        self.P = np.diag([1000.0, 1000.0, 1000.0, 1000.0, 1000.0])
        #print('P = ')
        #print(self.P, self.P.shape)

        # Process Noise Covariance Matrix Q
        sCamera = 5  # assume 8.8m/s2 as maximum acceleration, forcing the thymio ROBOT
        sCourse  = 0.087222222 # assume 0.1rad/s as maximum turn rate for the thymio ROBOT
        sSpeed= 6.15 # assume 8.8m/s2 as maximum acceleration, forcing the thymio ROBOT
        sYawrate = 0.1  # assume 1.0rad/s2 as the maximum turn rate acceleration for the thymio ROBOT
        self.Q = np.diag([sCamera, sCamera, sCourse, sSpeed, sYawrate])
        #print('Q = ')
        #print(self.Q, self.Q.shape)

        # Measurement Noise Covariance R
        varCamera = 4 # Standard Deviation of Camera Measurement
        varrot = 0.05 # Standard Deviation of rotation Measurement
        varspeed = 6.15 # Standard Deviation of the speed measurement
        varyaw = 0.1 # Standard Deviation of the yawrate measurement
        self.R = np.diag([varCamera, varCamera, varrot, varspeed, varyaw])
        #print('R =')
        #print(self.R, self.R.shape)

        # Identity Matrix I
        self.I = np.eye(numstates)
        #print('I =')
        #print(self.I, self.I.shape)


        # Initial State Vector
        # The states are (px, py, fi, v, w) = ([mm], [mm], [rad],[mm/s],[rad/s])
        self.x = np.matrix([[int(robot_pos[0][0]), int(robot_pos[0][1]), float(robot_pos[1]), 0.0, 0.0]]).T
        #print('x =')
        #print(self.x, self.x.shape)


        # Initial measurement vector  (px, py, fi, v, w) = ([mm], [mm], [rad],[mm/s],[rad/s])  Will be mesured
        self.measurements = np.matrix([[int(robot_pos[0][0]), int(robot_pos[0][1]), float(robot_pos[1]), 0.0, 0.0]]).T
        # Lenth of the measurement
        #print('measurements = ')
        #print(self.measurements)

    def estimate(self, dt):
        
        # Copy variabel for better understanding of formulas
        #dt = self.dt
        P = self.P  
        Q = self.Q 
        R = self.R 
        I = self.I 
        x = self.x 


        # Time Update (Prediction)
        # ========================
        # Project the state ahead
        # see "Dynamic Matrix"
                                                    
        if np.abs(self.measurements[4])<0.0001: # Driving straight
            x[0] = x[0] + x[3]*dt * np.cos(x[2])
            x[1] = x[1] + x[3]*dt * np.sin(x[2])
            x[2] = x[2]
            x[3] = x[3]
            x[4] = 0.0000001 # avoid numerical issues in Jacobians
        else: # otherwise
            x[0] = x[0] + (x[3]/x[4]) * (np.sin(x[4]*dt+x[2]) - np.sin(x[2]))
            x[1] = x[1] + (x[3]/x[4]) * (-np.cos(x[4]*dt+x[2])+ np.cos(x[2]))
            x[2] = (x[2] + x[4]*dt + np.pi) % (2.0*np.pi) - np.pi
            x[3] = x[3]
            x[4] = x[4]
        

        # Calculate the Jacobian of the Dynamic Matrix A
        # see "Calculate the Jacobian of the Dynamic Matrix with respect to the state vector"
        a13 = float((x[3]/x[4]) * (np.cos(x[4]*dt+x[2]) - np.cos(x[2])))
        a14 = float((1.0/x[4]) * (np.sin(x[4]*dt+x[2]) - np.sin(x[2])))
        a15 = float((dt*x[3]/x[4])*np.cos(x[4]*dt+x[2]) - (x[3]/x[4]**2)*(np.sin(x[4]*dt+x[2]) - np.sin(x[2])))
        a23 = float((x[3]/x[4]) * (np.sin(x[4]*dt+x[2]) - np.sin(x[2])))
        a24 = float((1.0/x[4]) * (-np.cos(x[4]*dt+x[2]) + np.cos(x[2])))
        a25 = float((dt*x[3]/x[4])*np.sin(x[4]*dt+x[2]) - (x[3]/x[4]**2)*(-np.cos(x[4]*dt+x[2]) + np.cos(x[2])))
        JA = np.matrix([[1.0, 0.0, a13, a14, a15],
                        [0.0, 1.0, a23, a24, a25],
                        [0.0, 0.0, 1.0, 0.0, dt],
                        [0.0, 0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 1.0]])


        # Project the error covariance ahead
        P = JA*P*JA.T + Q

        # Measurement Update (Correction)
        # ===============================
    
        
        if self.camera_avilable: # with 10Hz, every 5th step
            JH = np.diag([1.0, 1.0, 1.0, 1.0, 1.0])
        else: # every other step
            JH = np.diag([0.0, 0.0, 0.0, 1.0, 1.0])  

        # Calculating Kalman gain     
        S = JH*P*JH.T + R
        K = (P*JH.T) * np.linalg.inv(S)

        # Update the estimate via
        #Z = self.measurements.reshape(JH.shape[0],1)
        y = self.measurements - (JH*x)                         
        x = x + (K*y)

        # Update the error covariance
        P = (I - (K*JH))*P

        # Past again the copied value which were changed
        self.P = P  
        self.I = I 
        self.x = x 

        # If there is no error, output True
        self.camera_avilable = True

        # Return the state vector
        output = [np.array([x.item(0), x.item(1)]), x.item(2), self.camera_avilable ,1]   # 1 in the end is just to not crash YUAN
        return output

    def update_measurements(self, camera_data, thymio_data):

        # Extract the information
        px = camera_data[0][0]
        py = camera_data[0][1]
        phi = camera_data[1]
        left_speed = speed_conversion_factor*thymio_data[1]
        right_speed = speed_conversion_factor*thymio_data[0]

        # Converting the left and right speed, to an average speed and yawrate
        speed = (left_speed + right_speed)/2
        yawrate = (right_speed - left_speed)/(robot_diameter) 

        # Store the information at the right place
        self.camera_avilable = camera_data[2]
        self.measurements = np.matrix([px, py, phi, speed, yawrate]).T

    


