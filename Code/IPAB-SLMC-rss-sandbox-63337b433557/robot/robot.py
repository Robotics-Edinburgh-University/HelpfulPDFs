#!/usr/bin/env python

import numpy
import math
import sys
import os

# Get path of to the toddler file... to use always relative paths
CURRENT_PATH = os.getcwd() 

# import sensing modules
sys.path.insert(0, CURRENT_PATH + '/../controller')
from Observer_Dynamics import Observer_Dynamics


class Robot:
     
    def __init__(self,dt):
        
        self.dt = dt
        
        ###
        # Intrisic parameters
        ###
        self.l = 0.001 #0.00001                     # parameter in order to control the robot analytically
        self.wheels_radius = 4.3          # wheels radius #unit:cm 
        self.wheels_base_lenght = 21.5    # wheel_base_length    
        self.gear_ratio = 3.0/25.0        # gear_ratio
        self.max_RPM_motor =  375.0        # maximum RPM of the robot motors
      	#original 375 RPM, converted in radian/s by multiply by 2pi and divided by 60 , Unit rad/s 
        self.max_angular_vel_motor = self.max_RPM_motor*2*math.pi/60.0   # maximum angular velocity of the motors
        
        # Observer module
        self.observer = Observer_Dynamics(self.dt, self.gear_ratio, self.max_angular_vel_motor, self.wheels_radius, self.wheels_base_lenght)
       
        
        ###
        # Extrinsic parameters
        ###
        # The current pose of the robot
        self.Current_Pose = [0.0, 0.0, 0.0]
        
        # The current frame of the robot
        self.Current_Frame = numpy.array( [[math.cos(self.Current_Pose[2]), -1*math.sin(self.Current_Pose[2]) , self.Current_Pose[0]], 
					   [math.sin(self.Current_Pose[2]), math.cos(self.Current_Pose[2]) , self.Current_Pose[1]],
					   [0.0 , 0.0 , 1.0]])
        
        
        # The goal position of the robot
        self.Goal_pose = [0.0, 0.0]
        
        # Keep track of the poses the robot has already passed
        self.trace = [self.Current_Pose]
        # Keep track of the motor commands on the robot that have already passed
        self.trace_motor_speeds = []
        
        # List of goal points in order to perform the desired trajectory
        self.goal_trajectory = [] 
        
        
        
    # update the current pose of the robot    
    def update_current_state(self):
        
        # log of the trace of the robot 
	self.trace.append(self.Current_Pose)  
        
        # update the current robot position 
        self.Current_Pose = self.observer.Compute_new_robot_pose(self.Current_Pose,self.trace_motor_speeds.pop())
        
        # new frame according to the new current pose
        self.Current_Frame = numpy.array( [[math.cos(self.Current_Pose[2]), -1*math.sin(self.Current_Pose[2]) , self.Current_Pose[0]], 
					   [math.sin(self.Current_Pose[2]), math.cos(self.Current_Pose[2]) , self.Current_Pose[1]],
					   [0.0 , 0.0 , 1.0]])
        
        #self.update_current_frame()
        
    # update the current frame of the robot    
    def update_current_frame(self, new_frame):
        
        self.Current_Frame = numpy.dot(self.Current_Frame,new_frame)
    
    
    # upadte the motor logging list
    def update_current_motor_speed(self,motor_speed):
        self.trace_motor_speeds.append(motor_speed) 
        
    # set the desired trajectory of the robot
    def set_desired_trajectory(self,traj):
	
	self.goal_trajectory = traj
	
    # Convert the unicycle dynami model to differencial drive 	
    def unicicle2differential_drive(self,unicycle):
		
	velocity,wmega =  unicycle
	
	wm = wmega*self.wheels_base_lenght / 2*self.wheels_radius  #*2   # the second *2 is for the two wheels per side
	vm = 2*velocity / 2*self.wheels_radius  # *2  # the second *2 is for the two wheels per side
      
        # command the motors of the robot
	ul = vm - wm
	ur = vm + wm
	print "linear_vel " , ul ,"   ",ur
	return  (ul,ur)
      
    # Convert the linear velocity on the wheels to the percentage input on the motors...  
    def convert_wheel_lin_vel_2_motor_speed(self,ul,ur):
        
        # Angular velocities of the wheels
        W_ang  = numpy.array([ul,ur]) /self.wheels_radius
        
	# convert to RPM
	W_RPM = (W_ang * 60)/(2*math.pi)
        
        # Gear box effect to compute motor RPM from Wheel RPM
        Motor_RPM = W_RPM / self.gear_ratio
	    
        # Finally the percentage of the motor speed
        Motor_speed = (Motor_RPM/self.max_RPM_motor) * 100
        
        if abs(Motor_speed[0]) > 100:
	   print "Motor_speed left high" , Motor_speed
	   Motor_speed[0] = numpy.sign(Motor_speed[0]) * 100
	    
        if abs(Motor_speed[1]) > 100:
	   print "Motor_speed right high" , Motor_speed
	   Motor_speed[1] = numpy.sign(Motor_speed[1]) * 100
	
	if abs(Motor_speed[0]) < 80:
	   print "Motor_speed left low " , Motor_speed
	   Motor_speed[0] = numpy.sign(Motor_speed[0]) * 80
	    
        if abs(Motor_speed[1]) < 80:
	   print "Motor_speed right low " , Motor_speed
	   Motor_speed[1] = numpy.sign(Motor_speed[1]) * 80
	   
        return Motor_speed
	