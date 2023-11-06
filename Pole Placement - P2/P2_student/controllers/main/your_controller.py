# Fill in the respective functions to implement the controller

# Import libraries
import numpy as np
from base_controller import BaseController
from scipy import signal, linalg
from util import *

integral_f = 0
preverr_f = 0

# CustomController class (inherits from BaseController)
class CustomController(BaseController):

    def __init__(self, trajectory):

        super().__init__(trajectory)

        # Define constants
        # These can be ignored in P1
        self.lr = 1.39
        self.lf = 1.55
        self.Ca = 20000
        self.Iz = 25854
        self.m = 1888.6
        self.g = 9.81
        # integral_f = 0
        # preverr_f = 0

        # Add additional member variables according to your need here.

    def update(self, timestep):
        
        def compute_pid_f(err, p, i, d, delT):
            global integral_f
            global preverr_f
            prop = p*err
            integral_f += err*delT
            intg = i*integral_f
            deriv = d*(err - preverr_f)/delT
            preverr_f = err
            return (prop + intg + deriv)

        trajectory = self.trajectory

        l_r = self.lr
        l_f = self.lf
        C_a = self.Ca
        I_z = self.Iz
        m = self.m
        g = self.g

        # Fetch the states from the BaseController method
        delT, X, Y, xdot, ydot, psi, psidot = super().getStates(timestep)

        # Design your controllers in the spaces below. 
        # Remember, your controllers will need to use the states
        # to calculate control inputs (F, delta). 

        # ---------------|Lateral Controller|-------------------------
        """
        Please design your lateral controller below.
        .
        
        """
        _, node = closestNode(X, Y, trajectory)
        look_ahead = 50
        
        if ((node + look_ahead) >= 8203):
            look_ahead = 0
            
        X_gt, Y_gt = trajectory[node + look_ahead, 0], trajectory[node + look_ahead, 1]
        psi_desired = np.arctan2((Y_gt - Y), (X_gt - X))
        psi_err = wrapToPi(psi_desired - psi)
        
        # C_a = 20000
        # I_z = 25854
        # l_f = 1.55
        # l_r = 1.39
        # m = 1888.6
        
        A = A = np.array([[0, 1, 0, 0], [0, -4*C_a/(m*xdot), 4*C_a/m, -2*C_a*(l_f - l_r)/(m*xdot)], [0, 0, 0, 1], [0, -2*C_a*(l_f - l_r)/(I_z*xdot), 2*C_a*(l_f - l_r)/(I_z), -2*C_a*(l_f**2 + l_r**2)/(I_z*xdot)]])
        B = np.array([[0, 0], [2*C_a/m, 0], [0, 0], [2*C_a*l_f/I_z, 0]])
        desired_poles = np.array([-15, -10, -0.45, 5])
        K = signal.place_poles(A, B[:,0].reshape(4,1), desired_poles)
        K = K.gain_matrix
        # print(K.shape)
        e1 = 0 #np.sqrt((X - X_gt)**2 + (Y - Y_gt)**2)
        e2 = psi_err
        e1_dot = ydot + xdot*e2
        
        e2_dot = psidot  #e2/delT
        
        state_vector = np.array([e1, e1_dot, e2, e2_dot]).reshape(4,1)
        delta = float(-K @ state_vector)
        # ---------------|Longitudinal Controller|-------------------------
        """
        Please design your longitudinal controller below.
        . Salvaging Longitudinal controller from P1
        """
        err_pos = np.sqrt((X - X_gt)**2 + (Y - Y_gt)**2)
        err_vel = err_pos/delT
        upd_F = compute_pid_f(err_vel, 6, 0, 0, delT)
        
        F = upd_F

        # Return all states and calculated control inputs (F, delta)
        return X, Y, xdot, ydot, psi, psidot, F, delta
