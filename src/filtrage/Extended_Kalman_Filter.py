import numpy as np
import matplotlib.pyplot as plt
from sympy import Symbol, symbols, Matrix, sin, cos


class Kalman():

    def __init__(self):       

        # Initailise Variables
        numstates = 5 # number of states (px, py, fi, v, w)
        self.camera_avilable = False
        self.dt = 1.0/50.0 # Sample Rate of the Measurements is 50Hz
        self.dtCamera=1.0/10.0 # Sample Rate of Camera is 10Hz
        self.P = 0
        self.Q = 0
        self.R = 0
        self.I = 0
        self.x = 0
        self.measurements = 0
    
        #Copy for easier formel representations
        dt = self.dt

        # Initial Uncertainty P0
        self.P = np.diag([1000.0, 1000.0, 1000.0, 1000.0, 1000.0])
        print('P = ')
        print(self.P, self.P.shape)

        # Process Noise Covariance Matrix Q
        sCamera     = 0.5*8.8*dt**2  # assume 8.8m/s2 as maximum acceleration, forcing the thymio ROBOT
        sCourse  = 0.1*dt # assume 0.1rad/s as maximum turn rate for the thymio ROBOT
        sVelocity= 8.8*dt # assume 8.8m/s2 as maximum acceleration, forcing the thymio ROBOT
        sYaw     = 1.0*dt # assume 1.0rad/s2 as the maximum turn rate acceleration for the thymio ROBOT
        self.Q = np.diag([sCamera**2, sCamera**2, sCourse**2, sVelocity**2, sYaw**2])
        print('Q = ')
        print(self.Q, self.Q.shape)

        # Measurement Noise Covariance R
        varCamera = 6.0 # Standard Deviation of Camera Measurement
        varrot = 0.1 # Standard Deviation of rotation Measurement
        varspeed = 1.0 # Standard Deviation of the speed measurement
        varyaw = 0.1 # Standard Deviation of the yawrate measurement
        self.R = np.diag([varCamera**2, varCamera**2, varrot**2, varspeed**2, varyaw**2])
        print('R =')
        print(self.R, self.R.shape)

        # Identity Matrix I
        self.I = np.eye(numstates)
        print('I =')
        print(self.I, self.I.shape)


        # Initial State Vector
        # The states are (px, py, fi, v, w) = ([mm], [mm], [rad],[mm/s],[rad/s])
        self.x = np.matrix([[90, 80, 45/180.0*np.pi, 1.1, 0.1]]).T
        print('x =')
        print(self.x, self.x.shape)
        #[np.array(x,y), angle, t/f, robot_length]
        #
        # Initial measurement vector  (px, py, fi, v, w) = ([mm], [mm], [rad],[mm/s],[rad/s])  Will be mesured
        self.measurements = np.matrix([[120, 130, 90/180.0*np.pi, 1, 0.2]]).T
        # Lenth of the measurement
        print('measurements = ')
        print(self.measurements)

        # Past again the copied value
        self.dt = dt

    def estimate(self, measurements):
        
        # Copy variabel for better understanding of formulas
        dt = self.dt
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
        print('x = ')
        print(x)

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
        # Measurement Function
        hx = np.matrix([[float(x[0])],[float(x[1])],[float(x[2])],[float(x[3])],[float(x[4])]])
        print('hx = ')
        print(hx)

        if self.camera_avilable: # with 10Hz, every 5th step
            JH = np.diag([1.0, 1.0, 1.0, 1.0, 1.0])
        else: # every other step
            JH = np.diag([0.0, 0.0, 0.0, 1.0, 1.0])  

        # Calculating Kalman gain     
        S = JH*P*JH.T + R
        K = (P*JH.T) * np.linalg.inv(S)

        # Update the estimate via
        #Z = self.measurements.reshape(JH.shape[0],1)
        y = self.measurements - (JH*hx)                         # I added JH .... not 100% sure
        x = x + (K*y)

        # Update the error covariance
        P = (I - (K*JH))*P

        # Past again the copied value which were changed
        self.P = P  
        self.I = I 
        self.x = x 

        # Return the state vector
        return self.x

    def update_measurements(self, robot_state, thymio_data):
        robot_x = robot_state[0][0]
        robot_y = robot_state[0][1]
        robot_phi = robot_state[1]
        speed = thymio_data[0]
        yawrate = thymio_data[1]
        self.camera_avilable = robot_state[2]
        self.measurements = [robot_x, robot_y, robot_phi, speed, yawrate]

    #def get_result(self):
    #    return self.result

