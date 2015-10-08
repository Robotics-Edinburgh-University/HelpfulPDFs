#!/usr/bin/env python

import math
import numpy

class Controller:
  
    # Go_to_goal_point commands the motors of the robot to reach a predefined position in the world
    def __init__(self,l,robot):
      
	self.l = l   
        self.robot = robot
        
    # In case we would like some parameters on the go
    def set_parameters(self,l):
        
        self.l = l
        
       
    # move the robot to the goal
    def execute(self,  dt ):
        #Executes avoidance behavior based on state and dt.
        #state --> the state of the robot and the goal
        #dt --> elapsed time
        #return --> unicycle model  [velocity, omega]
        
        # The goal:
        x_g, y_g = self.robot.Goal_pose
        
        # The robot:
        x_r, y_r, theta = self.robot.Current_Pose
 
        
        pos = numpy.array([x_r,y_r])
        goal_pose = numpy.array([x_g,y_g])
        u = goal_pose - pos
        
        R = numpy.array([[numpy.cos(-theta),-1*numpy.sin(-theta)],[numpy.sin(-theta),numpy.cos(-theta)]])
        
        L = numpy.array([[1,0],[0,1/self.l]])
        
        
        vw = numpy.dot( L , numpy.dot( R ,u)  )

        return [vw[0], vw[1]]
        
      