#!/usr/bin/env python

import math
import numpy

class Observer_Dynamics(object):
    
    # This class is made in order to provide the pose of the robot 
    def __init__(self):
        
        self.ang_velocity_motors = [0.0, 0.0] 
        self.Gear_ratio = 0.0
        self.dt = 0.0
	self.max_agular_vel_motor = 0.0
	
    
    def Compute_new_robot_pose(self):
        
        return Pose(x_ne,y_new, theta)
        
        
      