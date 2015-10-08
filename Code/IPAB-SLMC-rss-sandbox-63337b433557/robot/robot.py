#!/usr/bin/env python

import numpy
import sys

# Get path of to the toddler file... to use always relative paths
CURRENT_PATH = os.getcwd() 

# import sensing modules
sys.path.insert(0, CURRENT_PATH + '/../controller')
from Observer_Dynamics import Observer_Dynamics


class Robot:
     
    def __init__(self, params):
        
        ###
        # Intrisic parameters
        ###
        self.l = 0.01            # parameter in order to control the robot analytically
        self.wheels_radius       # wheels radius
        self.wheels_base_lenght  # wheel_base_length    
        
        ###
        # Extrinsic parameters
        ###
        
        # The current pose of the robot
        self.Current_Pose = [0.0, 0.0, 0.0]
        
        # The goal position of the robot
        self.Goal_pose = [0.0, 0.0]
        
        # Keep track of the poses the robot has already passed
        self.trace = []
        
        # List of goal points in order to perform the desired trajectory
        self.goal_trajectory = [] 
        
        
        
        
    # update the current pose of the robot    
    def update_current_state(self):
        
        # log of the trace of the robot 
	self.trace.append(self.Current_Pose)  
        
        # update the current robot position 
        self.Current_Pose = Observer_Dynamics.Compute_new_robot_pose()
        
    # set the desired trajectory of the robot
    def set_desired_trajectory(self):
	
	self.goal_trajectory = []
	
	
    # Convert the unicycle dynami model to differencial drive 	
    def unicicle2differential_drive(self,unicicle):
		
	(velocity,wmega) =  unicicle
	
	wm = wmega*self.wheels_base_lenght / 2*self.wheels_radius  #*2   # the second *2 is for the two wheels per side
	vm = 2*velocity / 2*self.wheels_radius  # *2  # the second *2 is for the two wheels per side
      
        # command the motors of the robot
	ur = vm + wm
	ul = vm - wm
	
	return (ur,ul)
      
      
      
     
     
	