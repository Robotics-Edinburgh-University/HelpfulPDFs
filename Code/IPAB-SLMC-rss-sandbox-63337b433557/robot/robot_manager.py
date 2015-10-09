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
    def Goal_reached(self,goal,general_vec_orient):
         
         goal_reached_flag = False
	 
	 #goal_reached = numpy.linalg.norm(numpy.array(goal[0:2]) - numpy.array(self.robot.Current_Pose[0:2]))  
         
         #current_robot_orientation = numpy.dot( numpy.array(self.robot.Current_Frame[0:2,0:2]) , numpy.array([1,0]) )
         #angular_error_nom = numpy.dot( numpy.array(goal), current_robot_orientation) 
	 #angular_error_denom = numpy.linalg.norm(  (numpy.array(goal) * numpy.linalg.norm(current_robot_orientation))  ) 
	 #if (angular_error_denom != 0.0):
	     #angular_error = angular_error_nom/angular_error_denom
	 #else:
	     #angular_error = 1.0
         
      	 goal_reached = numpy.linalg.norm(numpy.array(goal[0:2]) - numpy.array(self.robot.Current_Pose[0:2]))  
         
         
         #rot_vec = numpy.dot(numpy.array(goal[0:2]) , numpy.array([1,0]))
	 #print rot_vec 
	 #goal_vec = list(goal) + [1]
	 #local_goal_point = numpy.dot(numpy.linalg.inv(self.robot.Current_Frame),numpy.array(goal_vec))
	 #print "sssddd " ,local_goal_point,"  ", goal_vec
         goal_vec = general_vec_orient/ numpy.linalg.norm(general_vec_orient)
         print "dddd " , goal_vec
	 goal_theta = math.atan2(goal_vec[1],(goal_vec[0])) 
	 print goal_theta ," sdsdsds  " , self.robot.Current_Pose[2]
         
         angular_error = goal_theta - self.robot.Current_Pose[2]  #   the error .... /2
         print " angular_error ", angular_error , "Theta " , self.robot.Current_Pose[2] , "Theta Initial " , self.robot.trace[0][2]
         print " goal_reached ", goal_reached ,"  ", goal ," ",self.robot.Current_Pose 
         
         if goal_reached < 0.5 and  abs(angular_error) < 0.1: 
            goal_reached_flag = True
            print " goal_reached "
         return goal_reached_flag
       
       
    # Set the desired trajectory towards the goal
    def set_the_desired_trajectory(self): 
	
	
	
	#traj = [[1,0],[2,0],[3,0],[4,0],[5,0],[5,0],[6,0],[7,0],[8,0],[9,0],[10,0],[11,0],[12,0],[13,0],[14,0],[15,0],[16,0],[17,0],[18,0],[19,0],[20,0] ,[21,0],[22,0],[23,0],[24,0],[25,0],[26,0],[27,0],[28,0],[29,0],[30,0]] 
	
	
	#traj = [[0,1,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],
	#	[0,-1,1],[1,0,1],[1,0,1],[1,0,1]]
	
	traj = [[0,20,1], [0,-20,1],[0,-20,1],[0,-20,1] ]
	
	#traj = [[0,1,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],
	#        [0,1,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],
	#        [0,1,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],
	#        [0,1,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1],[1,0,1]] 
	
	#traj_theta = []
	#for every_p in traj:
	    #traj_theta.append( [every_p[0],every_p[1], math.atan2(every_p[1] - 0 , every_p[0]-1)] ) 
	
	#traj = numpy.array(traj)#/10.0

	self.robot.set_desired_trajectory(traj)
	
	
	
    # make it go to where we want
    def move_robot_to_goal(self):
        
        self.set_the_desired_trajectory()
        current_sum_orient_vec = numpy.array([0,0,0])
        for subpoint in self.robot.goal_trajectory:
	    count = 0 
	    current_sum_orient_vec += numpy.array(subpoint)
	    local_goal_point = numpy.dot(self.robot.Current_Frame,subpoint)
	    local_goal_pt = (local_goal_point[0], local_goal_point[1])
	    distance_to_goal = self.Goal_reached(local_goal_pt,current_sum_orient_vec) 
	    while (distance_to_goal == False):
	        v_w = self.controller.execute(self.robot.Current_Pose,local_goal_pt)
                u_wheels = self.robot.unicicle2differential_drive(v_w)
                ul,ur = u_wheels
                motor_speed = self.robot.convert_wheel_lin_vel_2_motor_speed(ul,ur)
	        self.robot.update_current_motor_speed(motor_speed)
	        self.IO.setMotors(-motor_speed[0],motor_speed[1])
	        self.robot.update_current_state()
	        distance_to_goal = self.Goal_reached(local_goal_pt,current_sum_orient_vec)
	        count += 1
	    print " number of interation ", count
		
	self.IO.setStatus('flash')

         
         
       
           