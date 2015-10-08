#!/usr/bin/env python

import math
import numpy

class Observer_Dynamics:
    
    # This class is made in order to provide the pose of the robot 
    def __init__(self,dt):
        
        self.ang_velocity_motors = [0.0, 0.0] 
        self.Gear_ratio = 3.0/25.0
        self.dt = 0.0	
	self.max_angular_vel_motor = 375.0*2*math.pi/60.0 
	#original 375 RPM, converted in radian/s by multiply by 2pi and divided by 60  
	self.wheel_radius = 4.3				#unit:cm   
	self.rWheel_vel = 0.0
	self.lWhell_vel = 0.0

    def Compute_wheel_vel(self,rMotor_spped,lMotor_speed):	#Motor_speed, in percent format, 100%,20%...
	
	#V=w*R=max_angular_velocity*Motor_speed*wheel_radius
	self.rWheel_vel = self.max_angular_vel_motor * rMotor_spped * self.wheel_radius
	self.lWhell_vel = self.max_angular_vel_motor * lMotor_speed * self.wheel_radius
    
    #Compute the estimated robot pose
    def Compute_new_robot_pose(self,time_step,rMotor_speed,lMotor_speed):
	
	Compute_wheel_vel(rMotor_speed,lMotor_speed)	#compute current velocity of wheels
	#Calculate disances travelled by both wheels	
	dr = self.rWheel_vel * time_step			
	dl = self.lWheel_vel * time_step        

	theta_dt = (dr-dl)/L
	theta_mid = theta + theta_dt/2
	dst = (dr+dl)/2
	x_dt = dst*cos(theta_mid)
	y_dt = dst*sin(theta_mid)

	theta_new = theta + theta_dt
	x_new = x + x_dt
	y_new = y + y_dt
        return Pose(x_new,y_new, (theta_new + pi)%(2*pi)-pi)
        
        
      
