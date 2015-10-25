#!/usr/bin/env python

import numpy
import numpy.linalg
import sys
import os
import time
import math
import pprint
import copy

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
from FindRoom import *

# import calibration modules
sys.path.insert(0, CURRENT_PATH + '/calibration')
from calibrate import calibration


class Robot_manager:
    def __init__(self, io, vision):

        self.dt = 0.001
        self.robot = Robot(self.dt)

        # Controller module
        self.controller = Controller(self.robot.l, self.dt)

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

        # Instance of the calibration class
        self.calibration = calibration()

        self.findRoom = FindRoomByColor()
        self.estimatedRooms = []
        self.i_found_a_collision = False


    def run_robot(self):

        self.execute_path('EXIT_D','I')
        #self.straight_robot_motion()
        #self.start_and_stay_in_the_room()
        #self.calibrate_turning_rate()
        #time.sleep(5)
        #print " calibration "
        #self.perform_90_degrees_turn_right()
        #time.sleep(1)
        #self.perform_90_degrees_turn_left()
        #self.start_and_stay_until_find_room()
        print " over \n"

        time.sleep(15)

    # excute the provided path
    def execute_path(self,start,end):

        self.compute_traj_to_goal(start,end)
        #pprint.pprint(self.robot.goal_trajectory)
        #raw_input("Ssss")
        self.move_the_fucking_robot_to_goal()
        self.IO.setMotors(0,0)


    # move always forward
    def straight_robot_motion(self):
        traj = self.straight_traj()
        self.set_the_desired_trajectory(traj)
        self.move_the_fucking_robot_to_goal()
        self.IO.setMotors(0,0)


    # move always forward
    def turn_left_360_robot_motion(self):
        for i in xrange(6):
            self.set_tranjectory_left()
            self.move_the_fucking_robot_to_goal()
        self.IO.setMotors(0,0)


    # follow the wall
    def counterwise_follow_wall_in_room_stop_at_right_edge(self):
        self.start_and_stay_in_the_room()

    def start_and_stay_until_find_room(self):

        enableVisionEvery = 4   #every 4 steps
        while(1):
            first_catch = False
            second_catch = False
            malakitsa = 0
            while (1):
                sonar = self.IO.getSensors()[6]
                print "sonar:", sonar
                if sonar < 60:
                    if first_catch:
                        first_catch = False
                        break
                    first_catch = True
                else:
                    first_catch = False
                self.set_tranjectory_left()
                self.move_the_fucking_robot_to_goal()
                if malakitsa%enableVisionEvery==0:
                    self.snapShot()
                malakitsa += 1
            # it found a wall in a room
            print "Wall Found!"
            malakitsa = 0
            while (1):
                print self.i_found_a_collision
                if self.i_found_a_collision == True:
                    self.i_found_a_collision = False
                    break
                self.set_tranjectory_straight()
                self.move_the_fucking_robot_to_goal()
                if malakitsa%enableVisionEvery==0:
                    self.snapShot()
                malakitsa += 1
            malakitsa = 0
            while (1):
                IR_right = self.IO.getSensors()[7]
                print "IR right:", IR_right
                if IR_right < 100:
                    break

                self.set_tranjectory_straight()
                self.move_the_fucking_robot_to_goal()
                self.i_found_a_collision = False
                if malakitsa%enableVisionEvery==0:
                    self.snapShot()
                malakitsa += 1
            print "Going out of the ROOM!"

    def start_and_stay_in_the_room(self):
        first_catch = False
        second_catch = False

        while (1):
            sonar = self.IO.getSensors()[6]
            print "sonar:", sonar
            if sonar < 60:
                if first_catch:
                    first_catch = False
                    break
                first_catch = True
            else:
                first_catch = False
            self.set_tranjectory_left()
            self.move_the_fucking_robot_to_goal()
        # it found a wall in a room
        print "Wall Found!"

        while (1):
            print self.i_found_a_collision
            if self.i_found_a_collision == True:
                self.i_found_a_collision = False
                break
            self.set_tranjectory_straight()
            self.move_the_fucking_robot_to_goal()

        while (1):
            IR_right = self.IO.getSensors()[7]
            print "IR right:", IR_right
            if IR_right < 100:
                break

            self.set_tranjectory_straight()
            self.move_the_fucking_robot_to_goal()
            self.i_found_a_collision = False
        print "Going out of the ROOM!"
        self.IO.setMotors(0, 0)
        time.sleep(15)

    def move_the_fucking_robot_to_goal(self):

        sensors_interuption = (0, 0)
        subpoint_counter = -1
        collision_loop = []
        visionCounter = 0
        for subpoint in self.robot.goal_trajectory:
            self.robot.reset_current_state()
            steps = 0
            subpoint_counter += 1

            # Inner while loop that that move the robot for every subpoint
            # ___LOOP_BEGIN_________________________________________________________
            distance_to_goal = self.Goal_reached1(subpoint)
            while (distance_to_goal == False):

                # Collision While
                if len(collision_loop) > 0:

                    if subpoint[0] != 0 and steps >= 7:  # if collision in x (moving)
                        #print "COLLISION IN X(COLLISION LOOP)"
                        sensors_interuption = self.overall_sensors_direction()
                        # print "sensors : " , sensors_interuption
                        if sensors_interuption != (0, 0):
                            # collision_loop.append(1)			#if we have collision append
                            x = sensors_interuption[0]
                            y = sensors_interuption[1]
                            # if x != 0:
                            # collision_loop.append(1)
                            # self.robot.goal_trajectory.insert(subpoint_counter+1,[x * 5 ,0,1])
                            if y != 0:
                            #    print "X_COLLISION_LOOP:",self.distance_sensors.analogs_sensors[7]
                                if self.distance_sensors.analogs_sensors[7] >= self.distance_sensors.right_IR_limit:
                                    self.i_found_a_collision = True
                                collision_loop.append(1)
                                # if x!=0:
                                # self.robot.goal_trajectory.insert(subpoint_counter+2,[0, y * 0.001,1])
                                # else:
                                self.robot.goal_trajectory.insert(subpoint_counter + 1, [0, y * 0.001, 1])
                            if y == 0 and x == -1:
                                collision_loop.append(1)
                                self.robot.goal_trajectory.insert(subpoint_counter + 1, [x * 5, 0, 1])

                        collision_loop.pop()
                        break  # break the loop and continue

                    elif subpoint[1] != 0 and steps >= 20:  # if collision in y (rotating)
                        #print "COLLISION IN Y(COLLISION LOOP)"
                        sensors_interuption = self.overall_sensors_direction()
                        # print "sensors : " , sensors_interuption
                        if sensors_interuption != (0, 0):
                            # collision_loop.append(1)			#if we have collision append
                            x = sensors_interuption[0]
                            y = sensors_interuption[1]
                            # if x != 0:
                            #    collision_loop.append(1)
                            #   self.robot.goal_trajectory.insert(subpoint_counter+1,[x * 5 ,0,1])
                            if y != 0:
                             #   print "Y_COLLISION_LOOP:",self.distance_sensors.analogs_sensors[7]
                                if self.distance_sensors.analogs_sensors[7] >= self.distance_sensors.right_IR_limit:
                                    self.i_found_a_collision = True
                                collision_loop.append(1)
                                # if x!=0:
                                #    self.robot.goal_trajectory.insert(subpoint_counter+2,[0, y * 0.001,1])
                                # else:
                                self.robot.goal_trajectory.insert(subpoint_counter + 1, [0, y * 0.001, 1])
                            if y == 0 and x == -1:
                                collision_loop.append(1)
                                self.robot.goal_trajectory.insert(subpoint_counter + 1, [x * 5, 0, 1])

                        collision_loop.pop()
                        break  # break the loop and continue

                    else:
                        #print "Entered Collision LOOP No Next Collision"
                        step_point = subpoint[0:2]
                    # Normal While
                else:
                    sensors_interuption = self.overall_sensors_direction()
                    # print "sensors : " , sensors_interuption
                    if sensors_interuption != (0, 0):
                        #print "COLLISION FROM NORMAL LOOP"
                        # collision_loop.append(1)			#if we have collision append
                        x = sensors_interuption[0]
                        y = sensors_interuption[1]
                        # if x != 0:
                        #    collision_loop.append(1)
                        #    self.robot.goal_trajectory.insert(subpoint_counter+1,[x * 5 ,0,1])
                        if y != 0:
                            #print "NORMAL_LOOP:",self.distance_sensors.analogs_sensors[7]
                            if self.distance_sensors.analogs_sensors[7] >= self.distance_sensors.right_IR_limit:
                                self.i_found_a_collision = True
                            collision_loop.append(1)
                            #   if x!=0:
                            #       self.robot.goal_trajectory.insert(subpoint_counter+2,[0, y * 0.001,1])
                            #   else:
                            self.robot.goal_trajectory.insert(subpoint_counter + 1, [0, y * 0.001, 1])
                        if y == 0 and x == -1:
                            collision_loop.append(1)
                            self.robot.goal_trajectory.insert(subpoint_counter + 1, [x * 5, 0, 1])
                        break  # break the loop and continue
                    else:
                        step_point = subpoint[0:2]
                    # ___LOOP_END_________________________________________________________

                distance_to_goal = self.dynamics_control_motors(step_point, subpoint)
                steps += 1

  #          if visionCounter % 2 == 0:
  #              if len(self.retrieve_image()) > 0:
  #                  # To engage the servo motor
  #                  self.IO.servoEngage()
  #                  self.IO.setMotors(0,0)
  #                  self.IO.servoSet(0)
  #                  time.sleep(1)
  #                  self.IO.servoSet(90)
  #                  time.sleep(1)
  #                  self.IO.servoSet(0)
  #                  time.sleep(1)
  #          visionCounter += 1

        self.IO.setStatus('flash')
    def snapShot(self):

        latestRoomsFromColor = self.findRoom.returnRoom(self.retrieve_image())

        self.estimatedRooms = self.findRoom.update_rooms(self.estimatedRooms,latestRoomsFromColor)
        if len(self.estimatedRooms) == 1:
            print "FOUND ROOM:",self.estimatedRooms
            # To engage the servo motor
            self.IO.servoEngage()
            self.IO.setMotors(0,0)
            self.IO.servoSet(0)
            time.sleep(1)
            self.IO.servoSet(90)
            time.sleep(1)
            self.IO.servoSet(0)
            time.sleep(20)
            self.estimatedRooms = []
    # handle dynamics, control and motors
    def dynamics_control_motors(self, step_point, subpoint):

        v_w = self.controller.execute(self.robot.Current_Pose, step_point[0:2])

        # from uni2diff
        u_wheels = self.robot.unicicle2differential_drive(v_w)
        ul, ur = u_wheels

        # from lin vel of wheels 2 motor command
        motor_speed = self.robot.convert_wheel_lin_vel_2_motor_speed(ul, ur)
        self.robot.update_current_motor_speed(motor_speed)
        self.IO.setMotors(-motor_speed[0], motor_speed[1])

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
            self.IO.setMotors(0,0)

        time2 = time.time()
        print "TIME: ", time2 - time1

        color_list = self.vision.object_detected_list

        return color_list

    # Is the robot at the goal
    def Goal_reached1(self, goal):  # ,general_vec_orient):

        goal_reached_flag = False

        # vector orientantion of the robot
        current_robot_orientation = numpy.dot(numpy.array(self.robot.Current_Frame[0:2, 0:2]), numpy.array([1, 0]))

        if (numpy.linalg.norm(numpy.array(goal[0:2])) != 0.0):
            normalised_goal = numpy.array(goal[0:2]) / numpy.linalg.norm(numpy.array(goal[0:2]))
        else:
            normalised_goal = 0.0

        angular_error_nom = numpy.dot(normalised_goal, current_robot_orientation)
        angular_error_denom = numpy.linalg.norm(
            (numpy.array(normalised_goal) * numpy.linalg.norm(current_robot_orientation)))
        if (angular_error_denom != 0.0):
            # print "angular_error_nom " , angular_error_nom
            # print "angular_error_denom " , angular_error_denom
            angular_error = angular_error_nom / angular_error_denom
        else:
            angular_error = 1.0

        # distance to goal
        goal_reached = numpy.linalg.norm(numpy.array(goal[0:2]) - numpy.array(self.robot.Current_Pose[0:2]))

        # print "goal_reached " , goal_reached , " goal " ,numpy.array(goal[0:2])  , " Current pose " ,self.robot.Current_Pose
        # print "angular_error ", angular_error


        if goal_reached < 0.5 and 0.95 < abs(angular_error) < 1.05:
            goal_reached_flag = True
        # print " goal_reached "
        return goal_reached_flag



        # Interrupt commands from the sensors

    def overall_sensors_direction(self):

        sensors_IR = self.distance_sensors.return_direction_IR_Sonar_Sensors()
        sensors_Button = self.button_sensors.return_direction_Button_Sensors()
        return tuple(map(sum, zip(sensors_IR, sensors_Button)))


        # ------------------------------------------------------------------------------------------------

        # ------------------------------------------------------------------------------------------------
        #   .................. path planning related function start ......................................
        # ------------------------------------------------------------------------------------------------

        # ------------------------------------------------------------------------------------------------
        # - clear the array from duplicated sequential values

    def remove_adjacent(self, nums):
        return [a for a, b in zip(nums, nums[1:] + [not nums[-1]]) if a != b]

        # clear no perfect x or y vectors

    def remove_no_casual_values(self, all_info_path):
        for info_step in all_info_path:
            if abs(info_step[2]) != 5.0:
                all_info_path.remove(info_step)
            else:
                if ((info_step[1])[0] == 0. or (info_step[1])[1] == 0.) != True:
                    all_info_path.remove(info_step)

        return all_info_path

        # compute the norm of all the step vectors

    def norms_per_step(self, relative_coord_path):
        norm_path = []
        for vector in relative_coord_path:
            norm_path.append(numpy.linalg.norm(vector))

        return norm_path

    # compute possible roation mats
    def compute_vector_rot_trans(self, step_path_list):

        rotations_list = []
        for i in xrange(len(step_path_list) - 1):
            start = step_path_list[i]
            end = step_path_list[i + 1]
            R = self.rotation_mat_between_2_vecs(start, end)
            rot_vec = numpy.dot(R, numpy.array([1., 0.]))
            rotations_list.append(rot_vec)
        return rotations_list

    # rotation matrix between 2 vectors
    def rotation_mat_between_2_vecs(self, vec1, vec2):

        vec1 = vec1 / numpy.linalg.norm(vec1)
        vec2 = vec2 / numpy.linalg.norm(vec2)

        x1 = vec1[0]
        y1 = vec1[1]
        x2 = vec2[0]
        y2 = vec2[1]

        R = numpy.array([[(x1 * x2 + y1 * y2), -(x1 * y2 - x2 * y1)], [(x1 * y2 - x2 * y1), (x1 * x2 + y1 * y2)]])
        return R

    # duplicate all turns in order ro achive the 90 degrees turn
    def duplicate_turns(self, commands):

        duplicated_list = []
        for index, every_command in enumerate(commands):
            if every_command[0] == 0.0:
                duplicated_list.append((index, every_command))

        for num_additions, copied in enumerate(duplicated_list):
            commands.insert(copied[0] + num_additions, copied[1])

        return commands

    # Set the desired trajectory towards the goal
    def set_the_desired_trajectory(self, traj):

        self.robot.set_desired_trajectory(traj)

    # make all the trajectory
    def compute_traj_to_goal(self, waypoint_1, waypoint_2):
        # path from graph based path planner 
        waypoints_path_and_priority = self.retrieve_trajectory_waypone_to_wayptwo(waypoint_1, waypoint_2)

        # interpolated traj following the path
        traj = self.interpolate_the_trajectory(waypoints_path_and_priority)


        # ---- More generic solution to go from any point to a waypoint --------------------------------#
        # -----------------------------------------------------------------------------------------------#
        # compute vector from current pose to first waypoint and the interpolated trajectory with priority
        # self.robot.Current pose!!!
        # example!!!!!
        #waypoint_list = self.map_representaion.robot_map.waypoints
        #start_waypoint = filter(lambda waypoint: waypoint['name'] == 'EXIT_F', waypoint_list)
        #start_pose = (start_waypoint[0])['coord']
        # self.robot.reset_current_state([start_pose[0],start_pose[1],0.0])
        #first_step = self.retrieve_trajectory_from_currentpos_to_waypoint(start_pose, traj)
        #complete_traj = first_step + traj
        # -----------------------------------------------------------------------------------------------#


        complete_traj = traj

        self.global_trajectory = complete_traj
        #print self.global_trajectory
        relative_trajectory = self.make_trajectory_from_global_relative(complete_traj)

        # extend the straight lines of the trajectory...
        longer_relative_trajectory = self.command_trajectory(relative_trajectory)
        #pprint.pprint(longer_relative_trajectory)
        #raw_input("dddddddd")
        final_relative_trajectory = self.duplicate_turns(longer_relative_trajectory)


        self.set_the_desired_trajectory(final_relative_trajectory)

    # Extend the lines of the trajectory ....
    def command_trajectory(self,commnas_list):

        # find the indices where the list hast to be cut
        indices = [i for i, x in enumerate(commnas_list) if x[0] == 0]
        indices_backup = copy.deepcopy(indices)
        indices.insert(0,0)
        indices.insert(len(indices),len(commnas_list))

        # build the sublists with the straight lines
        straight_commands = []
        for i in zip(indices, indices[1:]):
            straight_commands.append(commnas_list[i[0]+1:i[1]])

        # extend the lists appropriately * 0.3
        for line in straight_commands:
            additional_straight = int(len(line)*0.35)
            line += [numpy.array([ 5.,  0.])]*additional_straight

        # merge the staight line lists with the turns
        for index, i in enumerate(indices_backup):
            straight_commands.insert(index+1+index,[commnas_list[i]])

        # flatten the final list
        final_plus_commands = [item for sublist in straight_commands for item in sublist]

        #pprint.pprint(final_plus_commands)

        return final_plus_commands


    # build the command list for the robot
    def make_trajectory_from_global_relative(self, traj):

        global_points = traj

        clear_g_points = self.remove_adjacent(global_points)

        relative_coord_path = []
        for i in xrange(len(clear_g_points) - 1):
            relative_coord_path.append(numpy.array(clear_g_points[i + 1]) - numpy.array(clear_g_points[i]))

        norm_path = self.norms_per_step(relative_coord_path)
        all_info_path = zip(clear_g_points, relative_coord_path, norm_path)
        all_info_path = self.remove_no_casual_values(all_info_path)

        # retrieve clear step list
        step_path_list = list(zip(*all_info_path)[1])

        # build the step commands!!! Ready
        rot_list = self.compute_vector_rot_trans(step_path_list)
        commands_list = list(numpy.array(rot_list) * numpy.array([5., 0.001]))

        return commands_list

    # from current position to the first waypoint
    def retrieve_trajectory_from_currentpos_to_waypoint(self, current_pose, traj):

        vector_to_1st_waypoint = []
        vector_to_1st_waypoint.append(current_pose[0:2])
        vector_to_1st_waypoint.append(traj[0]) #- numpy.array(current_pose[0:2]))
        vector_to_1st_waypoint_tuple = []
        for i in vector_to_1st_waypoint:
            vector_to_1st_waypoint_tuple.append((i[0],i[1]))
        return self.interpolate_the_trajectory(vector_to_1st_waypoint_tuple, 'y')

    # returns the coordinates of the trajectory global  
    def retrieve_trajectory_waypone_to_wayptwo(self, waypoint_1, waypoint_2):

        solution_path = self.map_representaion.retrieve_shortest_path(waypoint_1, waypoint_2)

        return self.map_representaion.get_coord_and_priority(solution_path)
        #return self.map_representaion.path_coordinates(solution_path)

    # interpolate the trajectory stepwise
    # priority_axis - >>> defines in which axis are we going to move first 
    def interpolate_the_trajectory(self, trajectory_priority, priority_axis = None):

        trajectory_list = []
        trajectory, priority_list = zip(*trajectory_priority)
        trajectory = list(trajectory)
        priority_list = list(priority_list)
        for segments_index in xrange(len(trajectory) - 1):
            if (trajectory[segments_index])[0] <= (trajectory[segments_index + 1])[0]:
                xpoints = numpy.arange((trajectory[segments_index])[0], (trajectory[segments_index + 1])[0] + 1, 5)
            else:
                xpoints = numpy.arange((trajectory[segments_index])[0], (trajectory[segments_index + 1])[0] - 1, -5)
            if len(xpoints) == 0: xpoints = [(trajectory[segments_index])[0]]

            if (trajectory[segments_index])[1] < (trajectory[segments_index + 1])[1]:
                ypoints = numpy.arange((trajectory[segments_index])[1], (trajectory[segments_index + 1])[1] + 1, 5)
            else:
                ypoints = numpy.arange((trajectory[segments_index])[1], (trajectory[segments_index + 1])[1] - 1, -5)
            if len(ypoints) == 0: ypoints = [(trajectory[segments_index])[1]]

            traj = self.combine_interpolated_paths(xpoints, ypoints, priority_list[segments_index])
            trajectory_list += traj

        return trajectory_list

    # provide the gamma trajectory according to priority
    def combine_interpolated_paths(self, xpoints, ypoints, priority_axis):

        if priority_axis == 'x':

            ypart_x_traj = [ypoints[0]] * len(xpoints)
            first_trajectory = zip(xpoints, ypart_x_traj)
            xpart_y_traj = [xpoints[-1]] * len(ypoints)
            second_trajectory = zip(xpart_y_traj, ypoints)
            # print first_trajectory
            # print second_trajectory

        elif priority_axis == 'y':

            xpart_y_traj = [xpoints[0]] * len(ypoints)
            first_trajectory = zip(xpart_y_traj, ypoints)
            ypart_x_traj = [ypoints[-1]] * len(xpoints)
            second_trajectory = zip(xpoints, ypart_x_traj)

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

     # Set the desired trajectory to turn left one time
    def set_tranjectory_left(self):
        traj = [[0, 0.001, 1]]
        self.robot.set_desired_trajectory(traj)

    # Set the desired trajectory to turn right one time
    def set_tranjectory_right(self):
        traj = [[0, -0.001, 1]]
        self.robot.set_desired_trajectory(traj)

    # Set the desired trajectory to turn right one time
    def set_tranjectory_straight(self):
        traj = [[5, 0, 1]]
        self.robot.set_desired_trajectory(traj)



    def straight_traj(self):
        traj = [[5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1],
                [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1],
                [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1],
                [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1]]

        #         [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1],
        #         [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1],
        #         [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1],
        #         [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1],
        #         [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1],
        #         [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1],
        #         [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1],
        #         [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1],
        #         [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1],
        #         [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1],
        #         [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1],
        #         [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1],
        #         [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1],
        #         [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1],
        #         [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1],
        #         [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1],
        #         [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1],
        #         [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1],
        #         [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1]]
        #
        return traj

    # not used
    def turn_right_360(self):
        traj = [[0, -0.001, 1],[0, -0.001, 1],[0, -0.001, 1],[0, -0.001, 1]]
        return traj

    # not used
    def turn_left_360(self):
        traj = [[0, 0.001, 1],[0, 0.001, 1],[0, 0.001, 1],[0, 0.001, 1]]
        return traj


    #---------------------------------------------------------------------------------------------------------------------
    #------------------------------------------ Motion for measurements--------------------------------------------------
    #---------------------------------------------------------------------------------------------------------------------

    def move_robot_to_measure(self):
        #print self.robot.goal_trajectory
        sonar_360_measures= []
        steps = 0
        for subpoint in self.robot.goal_trajectory:
            self.robot.reset_current_state()
            # Inner while loop that that move the robot for every subpoint
            # ___LOOP_BEGIN_________________________________________________________
            distance_to_goal = self.Goal_reached1(subpoint)
            while (distance_to_goal == False):
                step_point = subpoint[0:2]

                self.distance_sensors.update_analog_sensors_meas()
                sonar_360_measures.append(self.distance_sensors.analogs_sensors[6])
                # Normal While
                distance_to_goal = self.dynamics_control_motors(step_point, subpoint)
                steps += 1

        return sonar_360_measures

    # turn robot 90 degrees after calibration
    def turn_robot_90_degrees(self,subpoint):
        steps = 0
        step_limit_90 = self.calibration.turn_360_steps/4
        while (steps < step_limit_90):
                step_point = subpoint[0:2]
                distance_to_goal = self.dynamics_control_motors(step_point, subpoint)
                steps += 1

    # calibration function
    def calibrate_turning_rate(self):

        calibration_data = self.turn_right_360_robot_motion_for_calibration()
        #print calibration_data
        self.calibration.calibrate_robot(calibration_data)

    # move always forward
    def turn_right_360_robot_motion_for_calibration(self):
        sonar_360_values = []
        steps = 0
        for i in xrange(20):
            self.set_tranjectory_right()
            sonar_360_values += self.move_robot_to_measure()
        self.IO.setMotors(0,0)
        return sonar_360_values

    # turn exactly 90 degrees right
    def perform_90_degrees_turn_right(self):
        self.set_tranjectory_right()
        self.turn_robot_90_degrees(self.robot.goal_trajectory[0])
        self.IO.setMotors(0,0)

    # turn exactly 90 degrees left
    def perform_90_degrees_turn_left(self):
        self.set_tranjectory_left()
        self.turn_robot_90_degrees(self.robot.goal_trajectory[0])
        self.IO.setMotors(0,0)
