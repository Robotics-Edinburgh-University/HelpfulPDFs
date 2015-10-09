#!/usr/bin/env python

import numpy
import numpy.linalg
import sys
import os
import time
import math

# Get path of to the toddler file... to use always relative paths
CURRENT_PATH = os.getcwd() 


# import control modules
sys.path.insert(0, CURRENT_PATH + '/controller')
from Controller_Dynamics import Controller

# import robot status modules
sys.path.insert(0, CURRENT_PATH + '/robot')
from robot import Robot



class Robot_manager:
     
    def __init__(self,io):
       
       
       self.dt = 0.001
       self.robot = Robot(self.dt)

       # Controller module
       self.controller = Controller(self.robot.l,self.dt)
       
       # IO
       self.IO = io
     
    # Is the robot at the goal
    def Goal_reached(self,goal):
         
         goal_reached_flag = False
	 
	 
         goal_reached = numpy.linalg.norm(numpy.array(goal[0:2]) - numpy.array(self.robot.Current_Pose[0:2]))  
         
         current_robot_orientation = numpy.dot( numpy.array(self.robot.Current_Frame[0:2,0:2]) , numpy.array([1,0]) )
         angular_error_nom = numpy.dot( numpy.array(goal), current_robot_orientation) 
	 angular_error_denom = numpy.linalg.norm(  (numpy.array(goal) * numpy.linalg.norm(current_robot_orientation))  ) 
	 angular_error = angular_error_nom/angular_error_denom
	 
         print " angular_error ", angular_error , "Theta " , self.robot.Current_Pose[2] , "Theta Initial " , self.robot.trace[0][2]
         print " goal_reached ", goal_reached ,"  ", goal ," ",self.robot.Current_Pose , 
         
         if goal_reached < 0.05 and 0.95 < angular_error < 1.05 : 
            goal_reached_flag = True
            print " goal_reached "
         return goal_reached_flag
       
       
    # Set the desired trajectory towards the goal
    def set_the_desired_trajectory(self): 
	
	#traj = [[2, 0], [4, 0], [6, 0], [8, 0], [10, 0], 
	        #[12, 0], [14, 0], [16, 0], [18, 0], [20, 0], 
	        #[22, 0], [24, 0], [26, 0], [28, 0], [30, 0], 
	        #[32, 0], [34, 0], [36, 0], [38, 0], [40, 0], 
	        #[42, 0], [44, 0], [46, 0], [48, 0], [50, 0],	
	        #[50, 2], [50, 4], [50, 6], [50, 8], [50, 10], 
	        #[50, 12], [50, 14], [50, 16], [50, 18], [50, 20], 
	        #[50, 22], [50, 24], [50, 26], [50, 28], [50, 30], 
	        #[50, 32], [50, 34], [50, 36], [50, 38], [50, 40], 
		#[50, 42], [50, 44], [50, 46], [50, 48]] 
	
	traj = [[0,0.1]] #[2,1] ]# ,[1,1]] #,[1,1] ]#,[4,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1]]
	
	#traj_theta = []
	#for every_p in traj:
	    #traj_theta.append( [every_p[0],every_p[1], math.atan2(every_p[1] - 0 , every_p[0]-1)] ) 
	
	#traj = numpy.array(traj)#/10.0

	self.robot.set_desired_trajectory(traj)
	
	
	
    # make it go to where we want
    def move_robot_to_goal(self):
       
        self.set_the_desired_trajectory()
        for subpoint in self.robot.goal_trajectory:
	    count = 0 
	    distance_to_goal = self.Goal_reached(subpoint) 
	    while (distance_to_goal == False):
	        #local_goal_pt = (subpoint[0], subpoint[1])
	        v_w = self.controller.execute(self.robot.Current_Pose,subpoint)
                u_wheels = self.robot.unicicle2differential_drive(v_w)
                ul,ur = u_wheels
                motor_speed = self.robot.convert_wheel_lin_vel_2_motor_speed(ul,ur)
	        self.robot.update_current_motor_speed(motor_speed)
	        self.IO.setMotors(-motor_speed[0],motor_speed[1])
	        #time.sleep(self.dt)
	        self.robot.update_current_state()
	        distance_to_goal = self.Goal_reached(subpoint)
	        count += 1
	    print " number of interation ", count
		
	self.IO.setStatus('flash')

         
         
       
           