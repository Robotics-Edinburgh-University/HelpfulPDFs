#!/usr/bin/env python

from controller import Controller
import math
import numpy

class Go_to_goal_point(object):
    # Go_to_goal_point commands the motors of the robot to reach a predefined position in the world
    def __init__(self, params):
        
        Controller.__init__(self,params)
        self.heading_angle = 0

    def set_parameters(self, params):
        
        self.l = 0.01
        
        
    # Let's overwrite this way:
    def get_heading(self, state):
        """Get the direction from the robot to the goal as a vector."""
        
        # The goal:
        x_g, y_g = state.goal.x, state.goal.y
        
        # The robot:
        x_r, y_r, theta = state.pose

        # Where is the goal in the robot's frame of reference?
        return (math.atan2(y_g - y_r, x_g - x_r) - theta + math.pi)%(2*math.pi) - math.pi
    
    def get_other_heading(self,state): 
      goal_angle = self.get_heading(state)
      return numpy.array([math.cos(goal_angle),math.sin(goal_angle),1])

    def get_heading_angle(self, state):
        """Get the heading angle in the world frame of reference."""
        
        #x_g, y_g = state.goal.x, state.goal.y
        #x_r, y_r, theta = state.pose
        heading = self.get_other_heading(state)

        return math.atan2(heading[1],heading[0])
        #End Week 3 Assigment        
    
    
    def execute(self, state, dt):
        #Executes avoidance behavior based on state and dt.
        #state --> the state of the robot and the goal
        #dt --> elapsed time
        #return --> unicycle model  [velocity, omega]
        
        # The goal:
        x_g, y_g = state.goal.x, state.goal.y
        
        # The robot:
        x_r, y_r, theta = state.pose
 
        
        pos = numpy.array([x_r,y_r])
        goal_pose = numpy.array([x_g,y_g])
        u = goal_pose - pos
        
        R = numpy.array([[numpy.cos(-theta),-1*numpy.sin(-theta)],[numpy.sin(-theta),numpy.cos(-theta)]])
        l = self.l
        L = numpy.array([[1,0],[0,1/l]])
        
        
        vw = numpy.dot( L , numpy.dot( R ,u)  )

        return [vw[0], vw[1]]
        
      