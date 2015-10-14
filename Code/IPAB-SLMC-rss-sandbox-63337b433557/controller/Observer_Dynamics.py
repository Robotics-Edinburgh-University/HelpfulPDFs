#!/usr/bin/env python

import math
import numpy

class Observer_Dynamics:
    
    # This class is made in order to provide the pose of the robot 
    def __init__(self,dt ,gear_ratio,max_angular_vel_motor,wheel_radius, wheels_base_lenght):
        
        self.Gear_ratio = gear_ratio
        self.dt = dt	
	self.max_angular_vel_motor = max_angular_vel_motor 
	self.wheel_radius = wheel_radius 				#unit:cm   
	self.wheels_base_lenght = wheels_base_lenght
	
    def Compute_wheel_vel(self,Motor_speeds):	#Motor_speed, in percent format, 100%,20%...
	
	#V=w*R=max_angular_velocity*Motor_speed*wheel_radius   * self.Gear_ratio 
	self.lWhell_vel = self.max_angular_vel_motor * (Motor_speeds[0]/100) * self.wheel_radius #* self.Gear_ratio # *4 
	self.rWheel_vel = self.max_angular_vel_motor * (Motor_speeds[1]/100) * self.wheel_radius #* self.Gear_ratio # *4
	
    #Compute the estimated robot pose
    def Compute_new_robot_pose(self,robot_pose ,Motor_speeds):
	
	x = robot_pose[0]
	y = robot_pose[1]
	theta = robot_pose[2]
	
	self.Compute_wheel_vel(Motor_speeds)	#compute current velocity of wheels
	#Calculate disances travelled by both wheels	
	dl = self.lWhell_vel * self.dt        
	dr = self.rWheel_vel * self.dt			
	
	theta_dt = (dr-dl)/self.wheels_base_lenght
	theta_mid = theta + theta_dt/2.0
	dst = (dr+dl)/2.0
	x_dt = dst*math.cos(theta_mid)
	y_dt = dst*math.sin(theta_mid)

	theta_new = theta + theta_dt/1.8   ### !!!!! tuning parameter of the observer !!! to rotate the amount we want..empirical value
	x_new = x + x_dt
	y_new = y + y_dt
        return [x_new,y_new, (theta_new + math.pi)%(2*math.pi)-math.pi]
        
        
      
