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

# import sensing modules
sys.path.insert(0, CURRENT_PATH + '/sensing')
from ButtonSensor import ButtonSensor
from LightSensors import LightSensors
from Distance_sensors import Distance_sensors

class Robot_manager:
     
    def __init__(self,io):
       
       
       self.dt = 0.001
       self.robot = Robot(self.dt)

       # Controller module
       self.controller = Controller(self.robot.l,self.dt)
       
       # IO
       self.IO = io
       
       # Various Sensor classes 
       self.button_sensors = ButtonSensor(self.IO)
       self.lightSensor = LightSensors(self.IO)
       self.distance_sensors = Distance_sensors(self.IO)  
     
    # Set the desired trajectory towards the goal
    def set_the_desired_trajectory(self): 
	
	# working!!!!
	#traj = [[0,5,1],[15,0,1],[0,-5,1],[15,0,1],[0,-5,1],[15,0,1],
		#[0,5,1],[15,0,1],[0,-5,1],[15,0,1],[0,-5,1],[15,0,1],
		#[0,5,1],[15,0,1],[0,-5,1],[15,0,1],[0,-5,1],[15,0,1],
		#[0,5,1],[15,0,1],[0,-5,1],[15,0,1],[0,-5,1],[15,0,1]] 
	
	#traj = [[0,-5,1],[0,-5,1],[0,-5,1],[0,-5,1],[0,-5,1],[0,-5,1],[0,-5,1]]
	
	# a 90 degrees turn with combination of to steps
	#traj = [[5,-5,1],[0,-5,1]]
        
        #traj = [[0,-1,1]]
        #traj = [[5,0.0,1]]
        
        #traj = [[5,0,1],[5,0,1],[5,0,1],[5,0,1],[0,-0.001,1],
	        #[5,0,1],[5,0,1],[5,0,1],[5,0,1],[0,-0.001,1],
		#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[0,-0.001,1],
		#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[0,-0.001,1],
		#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[0,0.001,1],
	        #[5,0,1],[5,0,1],[5,0,1],[5,0,1],[0,0.001,1],
		#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[0,0.001,1],
		#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[0,0.001,1],
		#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[0,-0.001,1],
	        #[5,0,1],[5,0,1],[5,0,1],[5,0,1],[0,-0.001,1],
		#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[0,-0.001,1],
		#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[0,-0.001,1],
		#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[0,0.001,1],
	        #[5,0,1],[5,0,1],[5,0,1],[5,0,1],[0,0.001,1],
		#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[0,0.001,1],
		#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[0,0.001,1]]
                
        
        
        #traj = [[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1]]
	
	traj = [[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
		[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
	        [5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
		[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
	       	[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1]]
	
	self.robot.set_desired_trajectory(traj)
	
    
    # make it go to where we want
    def move_robot_to_goal1(self):
        
        self.set_the_desired_trajectory()
        
        for subpoint in self.robot.goal_trajectory:
	    self.robot.reset_current_state()
            count = 0 
	    break_flag = False

	    # compute error in distance
	    distance_to_goal = self.Goal_reached1(subpoint) 
	    while (distance_to_goal == False):
	        
	        #sensors_interuption = self.overall_sensors_direction()
	        sensors_interuption =  (0,0) 
	        if (0,0) !=  sensors_interuption :
		   #step_point = [sensors_interuption[0] * 10 + subpoint[0],sensors_interuption[1] * 10 + subpoint[1], 1] 
  		   step_point = [sensors_interuption[0],sensors_interuption[1], 1] 
  		   print "interrupt "
		   break_flag = True
		else:
		   step_point = subpoint[0:2]
		    
	        # dynamics 
	        v_w = self.controller.execute(self.robot.Current_Pose, step_point[0:2] )
	        
                # from uni2diff
                u_wheels = self.robot.unicicle2differential_drive(v_w)
                ul,ur = u_wheels
                
                # from lin vel of wheels 2 motor command
                motor_speed = self.robot.convert_wheel_lin_vel_2_motor_speed(ul,ur)
	        self.robot.update_current_motor_speed(motor_speed)
	        self.IO.setMotors(-motor_speed[0],motor_speed[1])
	        
	        # position update
	        self.robot.update_current_state()
	        
	        # error computation
	        distance_to_goal = self.Goal_reached1(subpoint)
	        if break_flag:
		  self.robot.reset_current_state()
		  break_flag = False
		  continue
	        count += 1
	        
	        
	    print " number of interation ", count
	    
	self.IO.setStatus('flash')
	
    def move_the_fucking_robot_to_goal(self):
        
        self.set_the_desired_trajectory()
	sensors_interuption = (0,0)
	subpoint_counter = -1
	collision_loop = []
	
        for subpoint in self.robot.goal_trajectory:
	    self.robot.reset_current_state()
            steps = 0 
	    subpoint_counter += 1
   	    #print "steps " , count

	    # compute error in distance
	    distance_to_goal = self.Goal_reached1(subpoint) 
	    while (distance_to_goal == False):
	        
	        # Collision While
	        if len(collision_loop) > 0:
		    
		    if subpoint[0] != 0 and steps >=7:		#if collision in x (moving)
		        print "COLLISION IN X(COLLISION LOOP)"
                        sensors_interuption = self.overall_sensors_direction()
		    
	                if sensors_interuption != (0,0):
			   # collision_loop.append(1)			#if we have collision append
		            x = sensors_interuption[0]
		            y = sensors_interuption[1]
			    print x
                            #if x != 0:
			        #collision_loop.append(1)
		                #self.robot.goal_trajectory.insert(subpoint_counter+1,[x * 5 ,0,1])
		            if y!= 0:
			        collision_loop.append(1)
		                #if x!=0:
   	                            #self.robot.goal_trajectory.insert(subpoint_counter+2,[0, y * 0.001,1])
	                        #else:
		                self.robot.goal_trajectory.insert(subpoint_counter+1,[0, y * 0.001,1])
		        collision_loop.pop()
			break					# break the loop and continue
		    elif subpoint[1] != 0 and steps >= 20:	#if collision in y (rotating)
		        print "COLLISION IN Y(COLLISION LOOP)"
                        sensors_interuption = self.overall_sensors_direction()
		    
	                if sensors_interuption != (0,0):
			    #collision_loop.append(1)			#if we have collision append
		            x = sensors_interuption[0]
		            y = sensors_interuption[1]
			    print x
                           # if x != 0:
  			    #    collision_loop.append(1)
		             #   self.robot.goal_trajectory.insert(subpoint_counter+1,[x * 5 ,0,1])
		            if y!= 0:
			        collision_loop.append(1)
		                #if x!=0:
   	                        #    self.robot.goal_trajectory.insert(subpoint_counter+2,[0, y * 0.001,1])
	                        #else:
		                self.robot.goal_trajectory.insert(subpoint_counter+1,[0, y * 0.001,1])
		        collision_loop.pop()            
			break					# break the loop and continue		      
		    else:
		        print "Entered Collision LOOP No Next Collision"
		        print subpoint[0],"  ",subpoint[1]
		        step_point = subpoint[0:2]
		#Normal While        
		else:
		    		        
                    sensors_interuption = self.overall_sensors_direction()
		    
	            if sensors_interuption != (0,0):
  		        print "COLLISION FROM NORMAL LOOP"
		        #collision_loop.append(1)			#if we have collision append
		        x = sensors_interuption[0]
		        y = sensors_interuption[1]
			print x
                        #if x != 0:
			#    collision_loop.append(1)
		        #    self.robot.goal_trajectory.insert(subpoint_counter+1,[x * 5 ,0,1])
		        if y!= 0:
			    collision_loop.append(1)
		         #   if x!=0:
   	                 #       self.robot.goal_trajectory.insert(subpoint_counter+2,[0, y * 0.001,1])
	                 #   else:
		            self.robot.goal_trajectory.insert(subpoint_counter+1,[0, y * 0.001,1])
			break					# break the loop and continue	      
		    else:
		        step_point = subpoint[0:2]		      
		    
	        # dynamics 
		v_w = self.controller.execute(self.robot.Current_Pose, step_point[0:2] )
		 
		# from uni2diff
		u_wheels = self.robot.unicicle2differential_drive(v_w)
		ul,ur = u_wheels
		  
		# from lin vel of wheels 2 motor command
		motor_speed = self.robot.convert_wheel_lin_vel_2_motor_speed(ul,ur)
		self.robot.update_current_motor_speed(motor_speed)
		self.IO.setMotors(-motor_speed[0],motor_speed[1])
		  
		# position update
		self.robot.update_current_state()
		 
		# error computation
		distance_to_goal = self.Goal_reached1(subpoint)
		steps += 1                        
	    
	    #if break_flag:	      	
                #x = sensors_interuption[0]
                #y = sensors_interuption[1]
                #entered = False
                #if x != 0:
		    #self.robot.goal_trajectory.insert(subpoint_counter+1,[x * 8 ,0,1])
		    #entered = True
		#if y!= 0:
		    #if entered == True:
		        #entered  = False
	                #self.robot.goal_trajectory.insert(subpoint_counter+2,[0, y * 0.001,1])
	            #else:
		        #self.robot.goal_trajectory.insert(subpoint_counter+1,[0, y * 0.001,1])
	        #break_flag = False
	    
	    #print " number of interation ", count
	    
	self.IO.setStatus('flash') 	
	
	##############################################
    def move_robot_to_goal_aris(self):
        
        self.set_the_desired_trajectory()
        self.sensors_interuption =  (0,0) 	#Initialization
	break_flag = False
	
        for subpoint in self.robot.goal_trajectory:
	    self.robot.reset_current_state()
            count = 0 
	    
   	    print "steps " , count

	    # compute error in distance
	    distance_to_goal = self.Goal_reached1(subpoint) 
	    while (distance_to_goal == False):
	        
	        
                self.sensors_interuption = self.overall_sensors_direction()
                print " Outer ",  self.sensors_interuption
	        if (0,0) !=  self.sensors_interuption :
		    break_flag = True
		    print "COLLISION!"
		    break
		else:
		    step_point = subpoint[0:2]
		    
	        # dynamics 
	        v_w = self.controller.execute(self.robot.Current_Pose, step_point[0:2] )
	        
                # from uni2diff
                u_wheels = self.robot.unicicle2differential_drive(v_w)
                ul,ur = u_wheels
                
                # from lin vel of wheels 2 motor command
                motor_speed = self.robot.convert_wheel_lin_vel_2_motor_speed(ul,ur)
	        self.robot.update_current_motor_speed(motor_speed)
	        self.IO.setMotors(-motor_speed[0],motor_speed[1])
	        
	        # position update
	        self.robot.update_current_state()
	        
	        # error computation
	        distance_to_goal = self.Goal_reached1(subpoint)
	        count += 1
	    
	    if break_flag:
	        temp_robot_manager = Robot_manager(self.IO)
	        temp_robot_manager.move_robot_to_goal_aris_collision(self.sensors_interuption)
	        break_flag = False
	    
	    #print " number of interation ", count
	    
	self.IO.setStatus('flash')   
	
	
    def move_robot_to_goal_aris_collision(self,sensors_interuption):
        
	break_flag = False
	
	x = sensors_interuption[0]
	y = sensors_interuption[1]
	
	custom_traj = [[x * 8 ,0,1],[0, y * 0.001,1]]
    	    
    	for subpoint in custom_traj:
	    self.robot.reset_current_state()
            count = 0 
            print " In collision" , custom_traj
	    # compute error in distance
	    distance_to_goal = self.Goal_reached1(subpoint) 
	    while (distance_to_goal == False):
	        
	        print "In the while of collision"
                #self.sensors_interuption = self.overall_sensors_direction()
	        step_point = subpoint[0:2]
		    
	        # dynamics 
	        v_w = self.controller.execute(self.robot.Current_Pose, step_point[0:2] )
	        
                # from uni2diff
                u_wheels = self.robot.unicicle2differential_drive(v_w)
                ul,ur = u_wheels
                
                # from lin vel of wheels 2 motor command
                motor_speed = self.robot.convert_wheel_lin_vel_2_motor_speed(ul,ur)
	        self.robot.update_current_motor_speed(motor_speed)
	        self.IO.setMotors(-motor_speed[0],motor_speed[1])
	        
	        # position update
	        self.robot.update_current_state()
	        
	        # error computation
	        distance_to_goal = self.Goal_reached1(subpoint)
	        count += 1
	    
	    if break_flag:
	        #temp_robot_manager = Robot_manager(self.IO)
	        #temp_robot_manager.move_robot_to_goal_aris_collision(self.sensors_interuption)
		break_flag = False
	    print " number of interation ", count
	    
	self.IO.setStatus('flash')
	
	
    # Is the robot at the goal
    def Goal_reached1(self,goal): #,general_vec_orient):
               
         goal_reached_flag = False
                  
         # vector orientantion of the robot 
         current_robot_orientation = numpy.dot( numpy.array(self.robot.Current_Frame[0:2,0:2]) , numpy.array([1,0]) )
         
         if (numpy.linalg.norm(numpy.array(goal[0:2])) != 0.0):
             normalised_goal = numpy.array(goal[0:2])/ numpy.linalg.norm(numpy.array(goal[0:2]))
         else:
	     normalised_goal = 0.0
	     
         angular_error_nom = numpy.dot( normalised_goal, current_robot_orientation) 
	 angular_error_denom = numpy.linalg.norm(  (numpy.array(normalised_goal) * numpy.linalg.norm(current_robot_orientation))  ) 
	 if (angular_error_denom != 0.0):
	     #print "angular_error_nom " , angular_error_nom
	     #print "angular_error_denom " , angular_error_denom
	     angular_error = angular_error_nom/angular_error_denom
	 else:
	     angular_error = 1.0
        
	 # distance to goal 
         goal_reached = numpy.linalg.norm(numpy.array(goal[0:2]) - numpy.array(self.robot.Current_Pose[0:2]))  

         #print "goal_reached " , goal_reached , " goal " ,numpy.array(goal[0:2])  , " Current pose " ,self.robot.Current_Pose
         #print "angular_error ", angular_error
         
         
         if goal_reached < 0.5 and  0.95 < abs(angular_error) < 1.05: 
            goal_reached_flag = True
            #print " goal_reached "
         return goal_reached_flag         
       
       
    # Interrupt commands from the sensors    
    def overall_sensors_direction(self):

        sensors_IR = self.distance_sensors.return_direction_IR_Sonar_Sensors()
	sensors_Button = self.button_sensors.return_direction_Button_Sensors()            
        return tuple(map(sum,zip(sensors_IR,sensors_Button)))
	
	
    
