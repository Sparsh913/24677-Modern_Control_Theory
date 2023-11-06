# Fill in the respective functions to implement the controller

# Import libraries
import numpy as np
from base_controller import BaseController
from scipy import signal, linalg
from util import *

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

        # Add additional member variables according to your need here.
        self.P = 1
        self.I = 0.5
        self.D = 1
        self.integral_f = 0
        self.preverr_f = 0
        self.integral_d = 0
        self.preverr_d = 0
        

    def update(self, timestep):
    
        def compute_pid_f(err, p, i, d, delT):
            prop = p*err
            self.integral_f += err*delT
            intg = i*self.integral_f
            deriv = d*(err - self.preverr_f)/delT
            self.preverr_f = err
            return (prop + intg + deriv)
            
        def compute_pid_d(err, p, i, d, delT):
            prop = p*err
            self.integral_d += err*delT
            intg = i*self.integral_d
            deriv = d*(err - self.preverr_d)/delT
            self.preverr_d = err
            return (prop + intg + deriv)

        trajectory = self.trajectory

        lr = self.lr
        lf = self.lf
        Ca = self.Ca
        Iz = self.Iz
        m = self.m
        g = self.g
        
        # P = self.P
        # I = self.I
        # D = self.D

        # Fetch the states from the BaseController method
        delT, X, Y, xdot, ydot, psi, psidot = super().getStates(timestep)

        # Design your controllers in the spaces below. 
        # Remember, your controllers will need to use the states
        # to calculate control inputs (F, delta). 

        # ---------------|Lateral Controller|-------------------------
        """
        Please design your lateral controller below.
      
        """
        # bearing = self.getBearingInRad
        node = closestNode(X, Y, trajectory)
        # X_gt, Y_gt = trajectory[0], trajectory[0]
        # err_X, err_Y = X - X_gt, Y - Y_gt
        # err_pos = node[0]
        # err_fwd = np.zeros(10)
        
        # Initialization
        # upd_F = 1000
        # upd_delta = 0
        
        look_ahead = 90
        
        if (node[1]+look_ahead) >= 8203:
            look_ahead = 20
            
        X_gt, Y_gt = trajectory[node[1]+look_ahead, 0], trajectory[node[1]+look_ahead, 1]
        
        # for j,i in enumerate(range(node[1], node[1]+10)):
            # print("iter", j)
            # print("traj_idx", i)
            # tr = trajectory[i]
            # err = np.sqrt((X-tr[0])**2 + (Y-tr[1]))
            # err_fwd[j] = err
            
        # de = (err_fwd[-1] - err_fwd[0])/len(err_fwd)
        # err_pos = np.sqrt(err_X**2 + err_Y**2)
        # err_psi = psi - bearing
        
        psi_desired = np.arctan2((Y_gt - Y), (X_gt - X))
        psi_err = wrapToPi(psi_desired - psi)
        
        # if err_pos >= 4.8:
            # upd_F = P*err_pos + D*de
            
        # if psi_err >= np.pi/7:
            # upd_delta = -P*psi_err
        
        upd_delta = compute_pid_d(psi_err, 1, 0.02, 5, delT)
        
        
        delta = upd_delta
        # ---------------|Longitudinal Controller|-------------------------
        """
        Please design your longitudinal controller below.
      
        """
        err_pos = np.sqrt((X - X_gt)**2 + (Y - Y_gt)**2)
        err_vel = err_pos/delT
        upd_F = compute_pid_f(err_vel, 1.8, 0, 0, delT)
        
        F = upd_F

        # Return all states and calculated control inputs (F, delta)
        return X, Y, xdot, ydot, psi, psidot, F, delta
