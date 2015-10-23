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
     
    def __init__(self,io,vision):
       
       
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
       
       #Have the instance of the vision class
       self.vision = vision


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
	
   
	
    def move_the_fucking_robot_to_goal(self):
        
        self.set_the_desired_trajectory()
	sensors_interuption = (0,0)
	subpoint_counter = -1
	collision_loop = []
	visionCounter = 0
        for subpoint in self.robot.goal_trajectory:
	    self.robot.reset_current_state()
            steps = 0 
            
	    subpoint_counter += 1
	    
	    # Inner while loop that that move the robot for every subpoint	
	    #___LOOP_BEGIN_________________________________________________________
	    distance_to_goal = self.Goal_reached1(subpoint) 
	    while (distance_to_goal == False):
	       
	        # Collision While
	        if len(collision_loop) > 0:
		    
		    if subpoint[0] != 0 and steps >=7:		#if collision in x (moving)
		        print "COLLISION IN X(COLLISION LOOP)"
                        sensors_interuption = self.overall_sensors_direction()
			print "sensors : " , sensors_interuption
	                if sensors_interuption != (0,0):
			   # collision_loop.append(1)			#if we have collision append
		            x = sensors_interuption[0]
		            y = sensors_interuption[1]
                            #if x != 0:
			        #collision_loop.append(1)
		                #self.robot.goal_trajectory.insert(subpoint_counter+1,[x * 5 ,0,1])
		            if y!= 0:
			        collision_loop.append(1)
		                #if x!=0:
   	                            #self.robot.goal_trajectory.insert(subpoint_counter+2,[0, y * 0.001,1])
	                        #else:
		                self.robot.goal_trajectory.insert(subpoint_counter+1,[0, y * 0.001,1])
		            if y==0 and x==-1:
			        collision_loop.append(1)
			        self.robot.goal_trajectory.insert(subpoint_counter+1,[x * 5 ,0,1])
			        
		        collision_loop.pop()
			break					# break the loop and continue
			
		    elif subpoint[1] != 0 and steps >= 20:	#if collision in y (rotating)
		        print "COLLISION IN Y(COLLISION LOOP)"
                        sensors_interuption = self.overall_sensors_direction()
                        print "sensors : " , sensors_interuption
	                if sensors_interuption != (0,0):
			    #collision_loop.append(1)			#if we have collision append
		            x = sensors_interuption[0]
		            y = sensors_interuption[1]
                           # if x != 0:
  			    #    collision_loop.append(1)
		             #   self.robot.goal_trajectory.insert(subpoint_counter+1,[x * 5 ,0,1])
		            if y!= 0:
			        collision_loop.append(1)
		                #if x!=0:
   	                        #    self.robot.goal_trajectory.insert(subpoint_counter+2,[0, y * 0.001,1])
	                        #else:
		                self.robot.goal_trajectory.insert(subpoint_counter+1,[0, y * 0.001,1])
		            if y==0 and x==-1:
			        collision_loop.append(1)
			        self.robot.goal_trajectory.insert(subpoint_counter+1,[x * 5 ,0,1])
			        
		        collision_loop.pop()            
			break					# break the loop and continue
			
		    else:
		        print "Entered Collision LOOP No Next Collision"
		        step_point = subpoint[0:2]
		#Normal While        
		else:		    		        
                    sensors_interuption = self.overall_sensors_direction()
		    print "sensors : " , sensors_interuption
	            if sensors_interuption != (0,0):
  		        print "COLLISION FROM NORMAL LOOP"
		        #collision_loop.append(1)			#if we have collision append
		        x = sensors_interuption[0]
		        y = sensors_interuption[1]
                        #if x != 0:
			#    collision_loop.append(1)
		        #    self.robot.goal_trajectory.insert(subpoint_counter+1,[x * 5 ,0,1])
		        if y!= 0:
			    collision_loop.append(1)
		         #   if x!=0:
   	                 #       self.robot.goal_trajectory.insert(subpoint_counter+2,[0, y * 0.001,1])
	                 #   else:
		            self.robot.goal_trajectory.insert(subpoint_counter+1,[0, y * 0.001,1])
		        if y==0 and x==-1:
			    collision_loop.append(1)
			    self.robot.goal_trajectory.insert(subpoint_counter+1,[x * 5 ,0,1])
			break					# break the loop and continue	      
		    else:
		        step_point = subpoint[0:2]		      
		#___LOOP_END_________________________________________________________   
		
		distance_to_goal = self.dynamics_control_motors(step_point,subpoint) 
		steps += 1                        
	    
	    if visionCounter%4 == 0:
	        if self.retrieve_image():
		    
		    # To engage the servo motor
		    self.IO.servoEngage()

                    self.IO.servoSet(0)
                    time.sleep(1)
                    self.IO.servoSet(90)
                    time.sleep(1)
                    self.IO.servoSet(0)
                    time.sleep(1)
	    visionCounter += 1    
	    
	self.IO.setStatus('flash') 	
    
   
    
    # handle dynamics, control and motors
    def dynamics_control_motors(self,step_point,subpoint):
      
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
	
	return distance_to_goal
      
      

    def retrieve_image(self):
        
        vision_break_flag = False
        self.vision.detect_object = True
        time1 = time.time()
	while self.vision.detect_object:
	     #time.sleep(0.05)
	     self.IO.setMotors(0.0,0.0)
	     #print "wait for vision"
	time2 = time.time()
	print "TIME: ",time2-time1

	temp_obj_list_colors = self.vision.object_detected_list
	print " object list ", temp_obj_list_colors    
	if temp_obj_list_colors[1] > 0: 
           self.IO.setMotors(0.0,0.0)	 		
	   print "Targets found " , temp_obj_list_colors[1]
	   #time.sleep(5)
	   vision_break_flag = True 
        
        print vision_break_flag
	return vision_break_flag


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
	
	
    
