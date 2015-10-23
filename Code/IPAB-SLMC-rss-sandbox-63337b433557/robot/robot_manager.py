#!/usr/bin/env python

import numpy
import numpy.linalg
import sys
import os
import time
import math
import pprint

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

# import mapping modules
sys.path.insert(0, CURRENT_PATH + '/environment')
from map_representaion import map_representation


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
       
       # Have the instance of the vision class
       self.vision = vision
  
       # Instance of the map and the graph for path planning
       self.map_representaion = map_representation()	
       
    def run_robot(self):

        start = time.time()
        #self.compute_traj_to_goal('A','D')
        self.set_the_desired_trajectory()
        self.move_the_fucking_robot_to_goal()
	#print self.robot_manager.overall_sensors_direction()
	self.IO.setMotors(0,0)
	end = time.time()
	print "time for 16 steps" , end-start
	print "time per steps" , (end-start)/16 
	print " over \n"
	time.sleep(15)   
    
    def move_the_fucking_robot_to_goal(self):
        
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
			#print "sensors : " , sensors_interuption
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
                        #print "sensors : " , sensors_interuption
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
		    #print "sensors : " , sensors_interuption
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
	        if len(self.retrieve_image()) > 0 :
		  
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
      
      
    # interface with vision
    def retrieve_image(self):
        
        
        self.vision.detect_object = True
        time1 = time.time()
	while self.vision.detect_object:
	    self.IO.setMotors(0.0,0.0)
	     
	time2 = time.time()
	print "TIME: ",time2-time1

	color_list = self.vision.object_detected_list
	
	colors = []
	for color in color_list:
	    if color > 0:
	        colors.append(color)
	        print "Color found found:" , color

	return colors


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


     # ------------------------------------------------------------------------------------------------

     # ------------------------------------------------------------------------------------------------
     #   .................. path planning related function start ......................................
     # ------------------------------------------------------------------------------------------------
     
     # ------------------------------------------------------------------------------------------------
     # - clear the array from duplicated sequential values
    def remove_adjacent(self,nums):
     return [a for a,b in zip(nums, nums[1:]+[not nums[-1]]) if a != b]   
    
    # clear no perfect x or y vectors
    def remove_no_casual_values(self,all_info_path):
	for info_step in all_info_path:
	    if abs(info_step[2]) != 5.0:
	       all_info_path.remove(info_step)
	return all_info_path    
      
    # compute the norm of all the step vectors
    def norms_per_step(self,relative_coord_path):
        norm_path = []
        for vector in relative_coord_path:
	    norm_path.append(numpy.linalg.norm(vector))
	return norm_path
     
    # compute possible roation mats
    def compute_vector_rot_trans(self,step_path_list):
        
        rotations_list = []
        for i in xrange(len(step_path_list)-1): 
	    start = step_path_list[i]  
            end =  step_path_list[i+1]
	    R = self.rotation_mat_between_2_vecs(start,end) 
	    rot_vec = numpy.dot(R, numpy.array([1.,0.]))
	    rotations_list.append(rot_vec)
        return rotations_list
       
    # rotation matrix between 2 vectors
    def rotation_mat_between_2_vecs(self,vec1,vec2):
        
        vec1 = vec1/numpy.linalg.norm(vec1)
        vec2 = vec2/numpy.linalg.norm(vec2)
        
        x1 = vec1[0]
        y1 = vec1[1]
        x2 = vec2[0]
        y2 = vec2[1]
        
        R = numpy.array([[(x1*x2+y1*y2 ), -(x1*y2-x2*y1)],[ (x1*y2-x2*y1), (x1*x2+y1*y2)] ])	
        return R
    
    # duplicate all turns in order ro achive the 90 degrees turn
    def duplicate_turns(self, commands):
      
        duplicated_list = []
        for index,every_command in enumerate(commands):
	    if every_command[0] == 0.0:
	        duplicated_list.append((index,every_command))
	
	for num_additions,copied in enumerate(duplicated_list):
	    commands.insert(copied[0]+num_additions,copied[1])
        
        return commands
    
    # Set the desired trajectory towards the goal
    def set_the_desired_trajectory(self,traj=None): 
	
	#traj = [[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
		#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
	        #[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
		#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
	       	#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1],
        	#[5,0,1],[5,0,1],[5,0,1],[5,0,1],[5,0,1]]
	#pprint.pprint(traj)
	self.robot.set_desired_trajectory(traj)
    
    # make all the trajectory
    def compute_traj_to_goal(self,waypoint_1,waypoint_2):
        # path from graph based path planner 
        waypoints_path = self.retrieve_trajectory_waypone_to_wayptwo(waypoint_1,waypoint_2)
        # interpolated traj following the path 
        traj = self.interpolate_the_trajectory(waypoints_path,'x')
        
        # compute vector from current pose to first waypoint and the interpolated trajectory with priority
        #self.robot.Current pose!!!
        
        # example!!!!!
        waypoint_list = self.map_representaion.robot_map.waypoints 
        start_waypoint = filter( lambda waypoint: waypoint['name']=='B', waypoint_list ) 
        start_pose = (start_waypoint[0])['coord'] 
        #self.robot.reset_current_state([start_pose[0],start_pose[1],0.0])
        
        first_step = self.retrieve_trajectory_from_currentpos_to_waypoint(self.robot.Current_Pose, traj)
	
	
	complete_traj = first_step + traj

	self.global_trajectory = complete_traj
	
	relative_trajectory = self.make_trajectory_from_global_relative(complete_traj)
	
	final_relative_trajectory = self.duplicate_turns(relative_trajectory)
	self.set_the_desired_trajectory(final_relative_trajectory)
    
    # build the command list for the robot
    def make_trajectory_from_global_relative(self,traj):
                
        global_points = traj
        
        clear_g_points = self.remove_adjacent(global_points)
        
        relative_coord_path = []
	for i in xrange(len(clear_g_points)-1): 
	    relative_coord_path.append(numpy.array(clear_g_points[i+1]) - numpy.array(clear_g_points[i])) 
        
        norm_path = self.norms_per_step(relative_coord_path)
	all_info_path = zip(clear_g_points,relative_coord_path,norm_path)
	all_info_path = self.remove_no_casual_values(all_info_path)
	 
	# retrieve clear step list 
	step_path_list = list(zip(*all_info_path)[1])

        # build the step commands!!! Ready
        rot_list = self.compute_vector_rot_trans(step_path_list)
        commands_list = list(numpy.array(rot_list) * numpy.array([5.,0.001]))
        
        return commands_list
      
    
    # from current position to the first waypoint
    def retrieve_trajectory_from_currentpos_to_waypoint(self,current_pose,traj):
        
        vector_to_1st_waypoint = [current_pose[0:2]] + [traj[0] - numpy.array(current_pose[0:2])]
        return self.interpolate_the_trajectory(vector_to_1st_waypoint,'x')
        
    # returns the coordinates of the trajectory global  
    def retrieve_trajectory_waypone_to_wayptwo(self,waypoint_1,waypoint_2):	
      
        solution_path = self.map_representaion.retrieve_shortest_path(waypoint_1,waypoint_2)
        return self.map_representaion.path_coordinates(solution_path)
    
    # interpolate the trajectory stepwise
    # priority_axis - >>> defines in which axis are we going to move first 
    def interpolate_the_trajectory(self, trajectory,priority_axis):
        
        trajectory_list = []
        for segments_index in xrange(len(trajectory)-1):
	    if (trajectory[segments_index])[0] <= (trajectory[segments_index+1])[0]:
	        xpoints = numpy.arange((trajectory[segments_index])[0] ,(trajectory[segments_index+1])[0] + 1, 5 )  
            else: 
	        xpoints = numpy.arange((trajectory[segments_index])[0] ,(trajectory[segments_index+1])[0] - 1, -5 ) 
	    if len(xpoints) == 0 : xpoints = [(trajectory[segments_index])[0] ]
	    
	    if (trajectory[segments_index])[1] < (trajectory[segments_index+1])[1]:
                ypoints = numpy.arange((trajectory[segments_index])[1] ,(trajectory[segments_index+1])[1] + 1, 5 )  
            else:
	        ypoints = numpy.arange((trajectory[segments_index])[1] ,(trajectory[segments_index+1])[1] - 1, -5 )  
            if len(ypoints) == 0 : ypoints = [(trajectory[segments_index])[1] ]
            
            traj = self.combine_interpolated_paths(xpoints,ypoints,priority_axis)
            trajectory_list += traj
            
    	return trajectory_list
   
    # provide the gamma trajectory according to priority
    def combine_interpolated_paths(self,xpoints , ypoints ,priority_axis):
        
        if priority_axis == 'x':
	  
	   ypart_x_traj = [ypoints[0]] * len(xpoints)
	   first_trajectory = zip(xpoints,ypart_x_traj)
	   xpart_y_traj = [xpoints[-1]] * len(ypoints)
	   second_trajectory = zip(xpart_y_traj,ypoints)
	   #print first_trajectory
	   #print second_trajectory
	   
	elif priority_axis == 'y':
	  
           xpart_y_traj = [xpoints[0]] * len(ypoints)
           first_trajectory = zip(xpart_y_traj,ypoints)	   
           ypart_x_traj = [ypoints[-1]] * len(xpoints)
      	   second_trajectory = zip(xpoints,ypart_x_traj)
	   
	else:
	   print "Wrong axis was provided"
           first_trajectory = []
           second_trajectory = []

        return first_trajectory + second_trajectory      
    
    
    
     # ------------------------------------------------------------------------------------------------

     # ------------------------------------------------------------------------------------------------
     #   .................. path planning related function end ......................................
     # ------------------------------------------------------------------------------------------------
     
     # ------------------------------------------------------------------------------------------------ 
