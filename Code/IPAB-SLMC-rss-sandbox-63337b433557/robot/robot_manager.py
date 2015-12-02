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
        self.base_found = False
        self.floorLightMin = 80
        self.roomThatJustFound = ""
        self.observe_left = True
        self.distance_sensors.turn = 1
        self.finished_left_wall_following = True
        self.ariskots = False


    def run_robot(self):

        self.close_cage()
        #self.pass_the_center()
        #self.from_D_to_C_and_search()
        #self.from_F_base_to_E()

        #self.straight_robot_motion()
        #self.start_and_stay_in_the_room()
        #self.calibrate_turning_rate()
        #time.sleep(5)
        #print " calibration "
        #self.perform_90_degrees_turn_right()
        #time.sleep(1)
        #self.perform_90_degrees_turn_left()
       # self.close_cage()
        while(1):
            #ADVANCED!
            #room = self.start_and_stay_until_find_room()
            #print "Starting Path"
            #self.execute_path("EXIT_" + room, 'E')
            #print "I ARRIVED First!"
            #room = self.start_and_stay_until_find_room()
            #if room == "B":
            #    break
            #BASIC!
            # room = self.go_to_the_exit_spot("F")
            # print "Starting Path"
            # self.execute_path("EXIT_" + room, 'E')
            #
            # room = self.start_and_stay_until_find_room()
            # while( room != "B"):
            #     print "Starting Path"
            #     self.execute_path("EXIT_" + room, 'E')
            #     room = self.start_and_stay_until_find_room()

            self.move_F_to_E_getBox_goBack()
         #   self.checkForBoxTheo(1)
         #   self.set_tranjectory_straight()
         #   self.move_the_fucking_robot_to_goal()
#            self.observe_the_room() # ITS FOR THE  MILESTONE 2
        #print "Over I arrived!\n",room
        #time.sleep(120)
        # print "Starting Path"
        # self.execute_path("EXIT_" + room, 'G')
        # print "I ARRIVED second!"
        # room = self.start_and_stay_until_find_room()
        # print " over \n"

    #ARIS OVERPOWERD
    def move_F_to_E_getBox_goBack(self):
        self.set_tranjectory_straight_2()
        self.move_the_fucking_robot_to_goal()
        room = self.go_to_the_exit_spot("F")
        self.execute_path_with_hack("EXIT_" + room, 'H')


        self.perform_45_degrees_turn_right()
        self.perform_11_degrees_turn_right()
        self.perform_11_degrees_turn_right()
        self.IO.setMotors(0,0)
        print "go straight 10"
        result = self.checkForBoxTheo2(22)
        print "now start obersvation"
        self.IO.setMotors(0,0)
        #go right 90 degrees and check simultaniusly
        enter = True
        if result == 1:
            enter = False
        while(enter):
            result = self.observe_the_room_final()
            if result == 1:
                print;print "GOT IT! <-------------------------";print
                break
            print "OBSERVE AGAIN!!!!!!!!!!!!!!!!!!!!!"

        room = self.go_to_the_exit_spot_F_FROM_E("E")


        self.perform_11_degrees_turn_right()
        self.set_tranjectory_straight_6_steps()
        self.move_the_fucking_robot_to_goal()
        self.set_tranjectory_straight_6_steps()
        self.move_the_fucking_robot_to_goal()
        self.set_tranjectory_straight_6_steps()
        self.move_the_fucking_robot_to_goal()
        self.set_tranjectory_straight_6_steps()
        self.move_the_fucking_robot_to_goal()
        self.IO.setMotors(0, 0)

        #
        #
        # time.sleep(5)
        # time.sleep(2)
        print "GO HOME!"
        self.findAndGoToBase()
        print "we arrived!"
        self.set_tranjectory_back_from_base()
        self.move_the_fucking_robot_to_goal()
        self.IO.setMotors(0, 0)
        self.open_cage()
        self.set_tranjectory_back_from_base()
        self.move_the_fucking_robot_to_goal()
        self.set_tranjectory_back_from_base()
        self.move_the_fucking_robot_to_goal()
        self.IO.setMotors(0, 0)
        self.close_cage()
        time.sleep(100)

    def findAndGoToBase(self):
        result = 0
        while (result == 0):
            result = self.checkForBlackpatchTheo(150)
            if result == 0:
                self.set_tranjectory_left()
                self.move_the_fucking_robot_to_goal_less_turn()
                self.IO.setMotors(0, 0)
        #Here we know we are on base with the box!
        self.IO.setMotors(0, 0)



    def from_F_base_to_E(self):
        self.perform_11_degrees_turn_right()
        self.perform_11_degrees_turn_right()
        self.perform_45_degrees_turn_right()
        self.perform_11_degrees_turn_right()
        self.perform_11_degrees_turn_right()
        for i in xrange(7):
            self.set_tranjectory_straight_6_steps()
            self.move_the_fucking_robot_to_goal()
            self.IO.setMotors(0,0)
        self.execute_path_with_hack('EXIT_F','J')
        print "OVER"



    def from_D_to_C_and_search(self):
        self.execute_path_with_hack('EXIT_D','Z')
        for i in xrange(3):
            self.set_tranjectory_straight_6_steps()
            self.move_the_fucking_robot_to_goal()
            self.IO.setMotors(0,0)
        print "OVER"
        self.checkForBoxTheo(60)

    def pass_the_center(self):
        self.perform_11_degrees_turn_right()
        self.perform_11_degrees_turn_right()
        self.perform_11_degrees_turn_right()
        for i in xrange(15):
            self.set_tranjectory_straight_6_steps()
            self.move_the_fucking_robot_to_goal()
            self.IO.setMotors(0,0)


    # excute the provided path
    def execute_path(self,start,end):

        self.compute_traj_to_goal(start,end)
        #pprint.pprint(self.robot.goal_trajectory)
        #raw_input("Ssss")
        self.move_the_fucking_robot_to_goal()
        self.IO.setMotors(0,0)

     # excute the provided path
    def execute_path_with_hack(self,start,end):

        self.compute_traj_to_goal(start,end)
        #pprint.pprint(self.robot.goal_trajectory)
        #raw_input("Ssss")
        self.move_the_fucking_robot_to_goal_medium_turn()
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

    def checkForBoxTheo(self,distance):
        tsiaFoundObject = False
        tsiaiCounter = 0
        enteredTheo = False
        for i in range(distance):
            move_y,move_x = self.foundBoxFromTheo()
            # if we found a box far away walk on it to recognize it
          #  print "Y =",move_y
          #  print "X=",move_x
            if move_y != -999:
                enteredTheo = True
                # if move_x < 30 use only Tsiai algortithm
                if move_x<40:
                    # Just to calibrate again
                    if move_y != 0 and move_y != -999:
                        self.turn_for_box(move_y)
                        self.move_the_fucking_robot_to_goal_less_turn()
                        self.IO.setMotors(0, 0)
                    for j in range(20):
                        # With step 2 check with the tsiai algorithm
                        if j%1 == 0:
                            if j==4:
                                if tsiaFoundObject == False:
                                    # Start again observation
                                    return 0
                            #move_y,move_x = self.found_box_from_tsiai()
                            if tsiaFoundObject == True and tsiaiCounter >=3:
                                print "Checking from Theo..."
                                move_y,move_x = self.foundBoxFromTheo()
                            else:
                                print "Checking from Tsiai..."
                                move_y,move_x = self.found_box_from_tsiai_super()

                            if move_y != -999:
                                if move_y != 0:
                                    self.turn_for_box(move_y)
                                    self.move_the_fucking_robot_to_goal_less_turn()
                                    self.IO.setMotors(0, 0)
                                if tsiaFoundObject == False:
                                    tsiaFoundObject = True
                                    print "Lets open the CAGE!"
                                    self.open_cage()
                            # now start counting
                            if tsiaFoundObject == True:
                                tsiaiCounter += 1

                        print ":D"
                        if self.box_inside() and tsiaFoundObject == True:
                            tsiaFoundObject = False
                            tsiaiCounter = 0
                            print "BOX GRABBED :) !!!"
                            self.IO.setMotors(0, 0)
                            self.close_cage()
                            ###self.IO.servoDisengage()
                            return 1
                           # time.sleep(10000)
                        self.set_tranjectory_straight()
                        self.move_the_fucking_robot_to_goal()

                    self.IO.setMotors(0, 0)
                    self.close_cage()
                    if self.box_inside() and tsiaFoundObject == True:
                        print "BOX GRABBED!!!"
                        self.IO.setMotors(0, 0)
                        ###self.IO.servoDisengage()
                        return 1
                      #  time.sleep(10000)
                    tsiaFoundObject = False
                    tsiaiCounter = 0
                    # Start again obersvatioN!
                    return 0
            # else run again Theo algorithm
            if move_y != 0 and move_y != -999:
                self.turn_for_box(move_y)
                self.move_the_fucking_robot_to_goal_less_turn()
                self.IO.setMotors(0, 0)
            # Continue Observation
            if enteredTheo == False:
                return -1
            # Move again a bit
            self.set_tranjectory_straight()
            self.move_the_fucking_robot_to_goal()
      ###  print "Failed and try again!! :( :(( :( :( "

    def checkForBoxTheo2(self,distance):
        tsiaFoundObject = False
        tsiaiCounter = 0
        enteredTheo = False
        for i in range(distance):
            move_y,move_x = self.foundBoxFromTheo()
            # if we found a box far away walk on it to recognize it
          #  print "Y =",move_y
          #  print "X=",move_x
            if move_y != -999:
                enteredTheo = True
                # if move_x < 30 use only Tsiai algortithm
                if move_x<40:
                    # Just to calibrate again
                    if move_y != 0 and move_y != -999:
                        self.turn_for_box(move_y)
                        self.move_the_fucking_robot_to_goal_less_turn()
                        self.IO.setMotors(0, 0)
                    for j in range(20):
                        # With step 2 check with the tsiai algorithm
                        if j%1 == 0:
                            if j==4:
                                if tsiaFoundObject == False:
                                    # Start again observation
                                    return 0
                            #move_y,move_x = self.found_box_from_tsiai()
                            if tsiaFoundObject == True and tsiaiCounter >=3:
                                print "Checking from Theo..."
                                move_y,move_x = self.foundBoxFromTheo()
                            else:
                                print "Checking from Tsiai..."
                                move_y,move_x = self.found_box_from_tsiai_super()

                            if move_y != -999:
                                if move_y != 0:
                                    self.turn_for_box(move_y)
                                    self.move_the_fucking_robot_to_goal_less_turn()
                                    self.IO.setMotors(0, 0)
                                if tsiaFoundObject == False:
                                    tsiaFoundObject = True
                                    print "Lets open the CAGE!"
                                    self.open_cage()
                            # now start counting
                            if tsiaFoundObject == True:
                                tsiaiCounter += 1

                        print ":D"
                        if self.box_inside() and tsiaFoundObject == True:
                            tsiaFoundObject = False
                            tsiaiCounter = 0
                            print "BOX GRABBED :) !!!"
                            self.IO.setMotors(0, 0)
                            self.close_cage()
                            ###self.IO.servoDisengage()
                            return 1
                           # time.sleep(10000)
                        self.set_tranjectory_straight()
                        self.move_the_fucking_robot_to_goal()

                    self.IO.setMotors(0, 0)
                    self.close_cage()
                    if self.box_inside() and tsiaFoundObject == True:
                        print "BOX GRABBED!!!"
                        self.IO.setMotors(0, 0)
                        ###self.IO.servoDisengage()
                        return 1
                      #  time.sleep(10000)
                    tsiaFoundObject = False
                    tsiaiCounter = 0
                    # Start again obersvatioN!
                    return 0
            # else run again Theo algorithm
            if move_y != 0 and move_y != -999:
                self.turn_for_box(move_y)
                self.move_the_fucking_robot_to_goal_less_turn()
                self.IO.setMotors(0, 0)
            # Continue Observation
            # Move again a bit
            self.set_tranjectory_straight()
            self.move_the_fucking_robot_to_goal()

    def checkForBlackpatchTheo(self,distance):
        tsiaFoundObject = False

        for i in range(distance):
            move_y,move_x = self.foundBoxFromBlackpatch()
            light = self.IO.getSensors()[5]
            print "--- light sensor -->",light
            if light < self.floorLightMin:
                return 1

            if move_y != -999:

                if move_y != 0 and move_y != -999:
                    self.turn_for_box(move_y)
                    self.move_the_fucking_robot_to_goal_less_turn()
                    self.IO.setMotors(0, 0)

            # Move again a bit
                self.set_tranjectory_straight()
                self.move_the_fucking_robot_to_goal()
            else:
                return 0

    def aris_algorithm(self):
        move_y,move_x = self.found_box_from_tsiai()
        if move_y != -999:
            print "Box Found! Lets Grab it!"
            if move_y < 0:
                print "Rotating a bit Right.."
            elif move_y > 0:
                print "Rotating a bit Left.."
            else:
                print "No need for rotation"

            if move_y != 0:
                self.turn_for_box(move_y)
                self.move_the_fucking_robot_to_goal_less_turn()
            self.IO.setMotors(0, 0)
            print move_y
            print "Go Straight"
            self.open_cage()
            ##################<<<<<--------------------------------------
            if self.vision.box_from_close_algorithm:
                print "Box is Close..."
                for i in range(10):
                    self.set_tranjectory_straight()
                    self.move_the_fucking_robot_to_goal()

                    #check one time only after 1 move
                    if i == 0 or i == 1:
                        print "one more check"
                        move_y,move_x = self.found_box_from_tsiai()
                        if move_y != -999:
                            print "box found again"
                            if move_y != 0:
                                self.turn_for_box(move_y)
                                self.move_the_fucking_robot_to_goal_less_turn()
                                self.IO.setMotors(0, 0)
                    if self.box_inside():
                        print "BOX GRABBED!!!"
                        self.IO.setMotors(0, 0)
                        self.close_cage()
                        self.IO.servoDisengage()
                        time.sleep(10000)
                print "I suck i failed to grabb it.. :/ Let me go back a bit"

                self.set_tranjectory_straight_back()
                self.move_the_fucking_robot_to_goal()

                print "Let me try again.."
                for i in range(7):
                    self.set_tranjectory_straight()
                    self.move_the_fucking_robot_to_goal()

                    move_y,move_x = self.found_box_from_tsiai()
                    if move_y != -999:
                        if move_y != 0:
                            self.turn_for_box(move_y)
                            self.move_the_fucking_robot_to_goal_less_turn()
                            self.IO.setMotors(0, 0)

                    if self.box_inside():
                        print "BOX GRABBED!!!"
                        self.IO.setMotors(0, 0)
                        self.close_cage()
                        self.IO.servoDisengage()
                        time.sleep(10000)
                print "I failed again."
                self.close_cage()
                return False
            else:
                print "Box is Far..."
                for i in range(22):
                    self.set_tranjectory_straight()
                    self.move_the_fucking_robot_to_goal()

                    #check one time only after 1 move
                    if i%2 == 0:
                        print "check again"
                        move_y,move_x = self.found_box_from_tsiai()
                        if move_y != -999:
                            "box found again"
                            if move_y != 0:
                                self.turn_for_box(move_y)
                                self.move_the_fucking_robot_to_goal_less_turn()
                                self.IO.setMotors(0, 0)

                    if self.box_inside():
                        print "BOX GRABBED!!!"
                        self.IO.setMotors(0, 0)
                        self.close_cage()
                        self.IO.servoDisengage()
                        time.sleep(10000)
                print "I failed, sorry"
                self.close_cage()
                if self.box_inside():
                    print "BOX GRABBED!!!"
                    self.IO.setMotors(0, 0)
                    self.IO.servoDisengage()
                    time.sleep(10000)
                return False
        # if is True it just continue the observe_the_room algorithm
        return True

    def observe_the_room_final(self):

        checkEvery = 1
        while(True):

            if self.observe_left:
                print "Left check Starting!"
            else:
                print "Right check Starting!"
            first_catch = False
            second_catch = False
            if self.finished_left_wall_following:
                my_steps = 0
                while (1):
                    if my_steps%1 == 0:
                        check = self.checkForBoxTheo(60)
                        if check == 0:
                            self.i_found_a_collision = False
                            #self.observe_left = True
                            #self.distance_sensors.turn = 1
                            #self.finished_left_wall_following = True
                            # start again observation
                            return 0
                        if check == 1:
                            #BOX FOUND AND GRABBED
                            return 1

                    sonar = self.IO.getSensors()[6]
                    if sonar < 60:
                        if first_catch:
                            first_catch = False
                            break
                        first_catch = True
                    else:
                        first_catch = False
                    self.set_tranjectory_left()
                    self.move_the_fucking_robot_to_goal()
                    my_steps+=1
            else:
                my_steps = 0
                while (1):
                    #check here 2
                    #check at the start custom

                    if my_steps%1 == 0:
                        check = self.checkForBoxTheo(60)
                        if check == 0:
                            self.i_found_a_collision = False
                            #self.observe_left = True
                            #self.distance_sensors.turn = 1
                            #self.finished_left_wall_following = True
                            return 0
                        if check == 1:
                            #BOX FOUND AND GRABBED
                            return 1

                    sonar = self.IO.getSensors()[6]
                    if sonar < 60:
                        if first_catch:
                            first_catch = False
                            break
                        first_catch = True
                    else:
                        first_catch = False
                    self.set_tranjectory_right()
                    self.move_the_fucking_robot_to_goal()
                    my_steps+=1
            # it found a wall in a room
            print "Wall Found!"
            self.i_found_a_collision = False
            my_steps = 0
            while (1):
                #check here 3
                if my_steps%checkEvery == 0:
                    check = self.checkForBoxTheo(60)
                    if check == 0:
                        self.i_found_a_collision = False
                        #self.observe_left = True
                        #self.distance_sensors.turn = 1
                        #self.finished_left_wall_following = True
                        return 0
                    if check == 1:
                        #BOX FOUND AND GRABBED
                        return 1

                if self.i_found_a_collision == True:
                    self.i_found_a_collision = False
                    break
                self.set_tranjectory_straight()
                self.move_the_fucking_robot_to_goal()
                my_steps+=1

            if self.observe_left:
                print "RIGHT IR LOCKED!"
            else:
                print "LEFT IR LOCKED!"
            my_steps = 0
            while (1):
                #check here 4
                if my_steps%checkEvery == 0:
                    check = self.checkForBoxTheo(60)
                    if check == 0:
                        self.i_found_a_collision = False
                        #self.observe_left = True
                        #self.distance_sensors.turn = 1
                        #self.finished_left_wall_following = True
                        return 0
                    if check == 1:
                        #BOX FOUND AND GRABBED
                        return 1

                IR_right = self.IO.getSensors()[7]
                IR_left = self.IO.getSensors()[0]
               # print "IR right:", IR_right
                if self.observe_left:
                    if IR_right < 100:
                        break
                else:
                    if IR_left < 100:
                        break
                self.set_tranjectory_straight()
                self.move_the_fucking_robot_to_goal()
                self.i_found_a_collision = False
                my_steps+=1
            print "ouups Sorry! I go back again now! :/"
            if self.observe_left:
                self.observe_left = False
                self.distance_sensors.turn = -1
                self.finished_left_wall_following = True
            else:
                self.observe_left = True
                self.distance_sensors.turn = 1
                self.finished_left_wall_following = False

            self.IO.setMotors(0, 0)
            #time.sleep(4)










    def observe_the_room(self):

        checkEvery = 4
        while(True):

            if self.observe_left:
                print "Left check Starting!"
            else:
                print "Right check Starting!"
            first_catch = False
            second_catch = False
            if self.finished_left_wall_following:
                my_steps = 0
                while (1):
                    #Check here 1
                    #check at the start custom
                    if my_steps == 0 and self.ariskots == True:
                        for j in range(10):
                            self.set_tranjectory_left()
                            self.move_the_fucking_robot_to_goal_less_turn()
                            self.IO.setMotors(0, 0)
                            check = self.aris_algorithm()
                            if check == False:
                                self.i_found_a_collision = False
                                self.observe_left = True
                                self.distance_sensors.turn = 1
                                self.finished_left_wall_following = True
                                return
                        print "out"
                    if my_steps%1 == 0:
                        check = self.aris_algorithm()
                        if check == False:
                            self.i_found_a_collision = False
                            self.observe_left = True
                            self.distance_sensors.turn = 1
                            self.finished_left_wall_following = True
                            return
                    sonar = self.IO.getSensors()[6]
                    if sonar < 60:
                        if first_catch:
                            first_catch = False
                            break
                        first_catch = True
                    else:
                        first_catch = False
                    self.set_tranjectory_left()
                    self.move_the_fucking_robot_to_goal()
                    my_steps+=1
            else:
                my_steps = 0
                while (1):
                    #check here 2
                    #check at the start custom
                    if my_steps == 0 and self.ariskots == True:
                        for j in range(10):
                            self.set_tranjectory_right()
                            self.move_the_fucking_robot_to_goal_less_turn()
                            self.IO.setMotors(0, 0)
                            check = self.aris_algorithm()
                            if check == False:
                                self.i_found_a_collision = False
                                self.observe_left = True
                                self.distance_sensors.turn = 1
                                self.finished_left_wall_following = True
                                return
                        print "out"
                    if my_steps%1 == 0:
                        check = self.aris_algorithm()
                        if check == False:
                            self.i_found_a_collision = False
                            self.observe_left = True
                            self.distance_sensors.turn = 1
                            self.finished_left_wall_following = True
                            return
                    sonar = self.IO.getSensors()[6]
                    if sonar < 60:
                        if first_catch:
                            first_catch = False
                            break
                        first_catch = True
                    else:
                        first_catch = False
                    self.set_tranjectory_right()
                    self.move_the_fucking_robot_to_goal()
                    my_steps+=1
            self.ariskots = True
            # it found a wall in a room
            print "Wall Found!"
            self.i_found_a_collision = False
            my_steps = 0
            while (1):
                #check here 3
                if my_steps%checkEvery == 0:
                    check = self.aris_algorithm()
                    if check == False:
                        self.i_found_a_collision = False
                        self.observe_left = True
                        self.distance_sensors.turn = 1
                        self.finished_left_wall_following = True
                        return
                if self.i_found_a_collision == True:
                    self.i_found_a_collision = False
                    break
                self.set_tranjectory_straight()
                self.move_the_fucking_robot_to_goal()
                my_steps+=1

            if self.observe_left:
                print "RIGHT IR LOCKED!"
            else:
                print "LEFT IR LOCKED!"
            my_steps = 0
            while (1):
                #check here 4
                if my_steps%checkEvery == 0:
                    check = self.aris_algorithm()
                    if check == False:
                        self.i_found_a_collision = False
                        self.observe_left = True
                        self.distance_sensors.turn = 1
                        self.finished_left_wall_following = True
                        return
                IR_right = self.IO.getSensors()[7]
                IR_left = self.IO.getSensors()[0]
               # print "IR right:", IR_right
                if self.observe_left:
                    if IR_right < 100:
                        break
                else:
                    if IR_left < 100:
                        break
                self.set_tranjectory_straight()
                self.move_the_fucking_robot_to_goal()
                self.i_found_a_collision = False
                my_steps+=1
            print "ouups Sorry! I go back again now! :/"
            if self.observe_left:
                self.observe_left = False
                self.distance_sensors.turn = -1
                self.finished_left_wall_following = True
            else:
                self.observe_left = True
                self.distance_sensors.turn = 1
                self.finished_left_wall_following = False

            self.IO.setMotors(0, 0)
            #time.sleep(4)

    ###############3
    def start_and_stay_until_find_room(self):

        enableVisionEvery = 4   #every 4 steps
        room_found = False
        while(room_found == False):
            first_catch = False
            second_catch = False
            malakitsa = 0
            while (1):
                light = self.IO.getSensors()[5]
                print light
                if light < self.floorLightMin:
                    self.base_found = True
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
                if malakitsa%(enableVisionEvery/2)==0:
                    self.snapShot()
                    if self.roomThatJustFound!="": # if u find a room break
                        room_found = True
                        break
                malakitsa += 1
            if room_found == True:
                break
            # it found a wall in a room
            print "Wall Found!"
            malakitsa = 0
            self.i_found_a_collision = False
            while (1):
                light = self.IO.getSensors()[5]
                print light
                if light < self.floorLightMin:
                    self.base_found = True
                print self.i_found_a_collision
                if self.i_found_a_collision == True:
                    self.i_found_a_collision = False
                    break
                self.set_tranjectory_straight()
                self.move_the_fucking_robot_to_goal()
                if malakitsa%enableVisionEvery==0:
                    self.snapShot()
                    if self.roomThatJustFound!="": # if u find a room break
                        room_found = True
                        break
                malakitsa += 1
            if room_found == True:
                break
            print "IR Locked"
            malakitsa = 0
            while (1):
                light = self.IO.getSensors()[5]
                print light
                if light < self.floorLightMin:
                    self.base_found = True
                IR_right = self.IO.getSensors()[7]
            #    print "IR right:", IR_right
                if IR_right < 100:
                    break

                self.set_tranjectory_straight()
                self.move_the_fucking_robot_to_goal()
                self.i_found_a_collision = False
                if malakitsa%enableVisionEvery==0:
                    self.snapShot()
                    if self.roomThatJustFound!="": # if u find a room break
                        room_found = True
                        break
                malakitsa += 1
            if room_found == True:
                break
        #    print "Going out of the ROOM!"
        if self.roomThatJustFound!="":
            print "I AM IN ROOM: ",self.roomThatJustFound
            temp = self.roomThatJustFound
            self.roomThatJustFound = ""
            print "I GO TO THE EXIT SPOT ",temp
            return self.go_to_the_exit_spot(temp)
        else:
            print "BUG!!"
            return None


    def go_to_the_exit_spot(self,myroom):
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
       # print "Wall Found!"
        self.i_found_a_collision = False
        while (1):
            print self.i_found_a_collision
            if self.i_found_a_collision == True:
                self.i_found_a_collision = False
                break
            self.set_tranjectory_straight()
            self.move_the_fucking_robot_to_goal()

        while (1):
            IR_right = self.IO.getSensors()[7]
           # print "IR right:", IR_right
            if IR_right < 100:
                break

            self.set_tranjectory_straight()
            self.move_the_fucking_robot_to_goal()
            self.i_found_a_collision = False
        print "I AM IN THE EXIT SPOT ",myroom
        if myroom == "B" or myroom == "E":
            self.set_tranjectory_straight_6_steps()
            self.move_the_fucking_robot_to_goal()
        self.IO.setMotors(0, 0)
        return myroom
    ############################################################################
    ############################################################################
    def go_to_the_exit_spot_F_FROM_E(self,myroom):
        first_catch = False
        second_catch = False
        self.distance_sensors.turn = 1
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
            self.set_tranjectory_right()
            self.move_the_fucking_robot_to_goal()
        # it found a wall in a room
        print "Wall Found!"
        self.i_found_a_collision = False
        while (1):
            print self.i_found_a_collision
            if self.i_found_a_collision == True:
                self.i_found_a_collision = False
                break
            self.set_tranjectory_straight()
            self.move_the_fucking_robot_to_goal()

        while (1):
            IR_right = self.IO.getSensors()[7]
           # print "IR right:", IR_right
            if IR_right < 100:
                break

            self.set_tranjectory_straight()
            self.move_the_fucking_robot_to_goal()
            self.i_found_a_collision = False
        print "I AM IN THE EXIT SPOT ",myroom
        if myroom == "B" or myroom == "E":
            #self.set_tranjectory_straight_6_steps()

            for i in range(3):
                self.perform_11_degrees_turn_right()
                self.set_tranjectory_straight()
                self.move_the_fucking_robot_to_goal()
                self.IO.setMotors(0, 0)
                self.set_tranjectory_straight()
                self.move_the_fucking_robot_to_goal()
                self.IO.setMotors(0, 0)
                self.perform_11_degrees_turn_right()
                self.set_tranjectory_straight()
                self.move_the_fucking_robot_to_goal()
                self.IO.setMotors(0, 0)
                self.set_tranjectory_straight()
                self.move_the_fucking_robot_to_goal()
                self.IO.setMotors(0, 0)

        self.IO.setMotors(0, 0)
        return myroom
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
                            # FOR LEFT AS THE PREVIOUS IMPLEMENTATION
                                if self.observe_left:
                                    if y == 1:
                                        if self.distance_sensors.analogs_sensors[7] >= self.distance_sensors.right_IR_limit - 214:
                                            self.i_found_a_collision = True
                                    collision_loop.append(1)
                                    # if x!=0:
                                    # self.robot.goal_trajectory.insert(subpoint_counter+2,[0, y * 0.001,1])
                                    # else:
                                    self.robot.goal_trajectory.insert(subpoint_counter + 1, [0, y * 0.001, 1])
                            # FOR RIGHT A BIT DIFFERENT
                                else:
                                    if y == -1:
                                        if self.distance_sensors.analogs_sensors[0] >= self.distance_sensors.left_IR_limit - 214:
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
                                if self.observe_left:
                                    if y==1:
                                        if self.distance_sensors.analogs_sensors[7] >= self.distance_sensors.right_IR_limit - 214:
                                            self.i_found_a_collision = True
                                    collision_loop.append(1)
                                    # if x!=0:
                                    #    self.robot.goal_trajectory.insert(subpoint_counter+2,[0, y * 0.001,1])
                                    # else:
                                    self.robot.goal_trajectory.insert(subpoint_counter + 1, [0, y * 0.001, 1])
                                else:
                                    if y==-1:
                                        if self.distance_sensors.analogs_sensors[0] >= self.distance_sensors.left_IR_limit - 214:
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
                            if self.observe_left:
                                if y==1:
                                    if self.distance_sensors.analogs_sensors[7] >= self.distance_sensors.right_IR_limit - 214:
                                        self.i_found_a_collision = True
                                collision_loop.append(1)
                                #   if x!=0:
                                #       self.robot.goal_trajectory.insert(subpoint_counter+2,[0, y * 0.001,1])
                                #   else:
                                self.robot.goal_trajectory.insert(subpoint_counter + 1, [0, y * 0.001, 1])
                            else:
                                if y==-1:
                                    if self.distance_sensors.analogs_sensors[0] >= self.distance_sensors.left_IR_limit - 214:
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


        self.IO.setStatus('flash')

    def move_the_fucking_robot_to_goal_less_turn(self):

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
                if(steps >=18):
                    break
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
                            # FOR LEFT AS THE PREVIOUS IMPLEMENTATION
                                if self.observe_left:
                                    if y == 1:
                                        if self.distance_sensors.analogs_sensors[7] >= self.distance_sensors.right_IR_limit - 214:
                                            self.i_found_a_collision = True
                                    collision_loop.append(1)
                                    # if x!=0:
                                    # self.robot.goal_trajectory.insert(subpoint_counter+2,[0, y * 0.001,1])
                                    # else:
                                    self.robot.goal_trajectory.insert(subpoint_counter + 1, [0, y * 0.001, 1])
                            # FOR RIGHT A BIT DIFFERENT
                                else:
                                    if y == -1:
                                        if self.distance_sensors.analogs_sensors[0] >= self.distance_sensors.left_IR_limit - 214:
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
                                if self.observe_left:
                                    if y==1:
                                        if self.distance_sensors.analogs_sensors[7] >= self.distance_sensors.right_IR_limit - 214:
                                            self.i_found_a_collision = True
                                    collision_loop.append(1)
                                    # if x!=0:
                                    #    self.robot.goal_trajectory.insert(subpoint_counter+2,[0, y * 0.001,1])
                                    # else:
                                    self.robot.goal_trajectory.insert(subpoint_counter + 1, [0, y * 0.001, 1])
                                else:
                                    if y==-1:
                                        if self.distance_sensors.analogs_sensors[0] >= self.distance_sensors.left_IR_limit - 214:
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
                            if self.observe_left:
                                if y==1:
                                    if self.distance_sensors.analogs_sensors[7] >= self.distance_sensors.right_IR_limit - 214:
                                        self.i_found_a_collision = True
                                collision_loop.append(1)
                                #   if x!=0:
                                #       self.robot.goal_trajectory.insert(subpoint_counter+2,[0, y * 0.001,1])
                                #   else:
                                self.robot.goal_trajectory.insert(subpoint_counter + 1, [0, y * 0.001, 1])
                            else:
                                if y==-1:
                                    if self.distance_sensors.analogs_sensors[0] >= self.distance_sensors.left_IR_limit - 214:
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

    def move_the_fucking_robot_to_goal_medium_turn(self):

        sensors_interuption = (0, 0)
        subpoint_counter = -1
        collision_loop = []
        visionCounter = 0
        step_limit_22_medium = self.calibration.turn_360_steps/16
        for subpoint in self.robot.goal_trajectory:
            self.robot.reset_current_state()
            steps = 0
            subpoint_counter += 1

            # Inner while loop that that move the robot for every subpoint
            # ___LOOP_BEGIN_________________________________________________________
            distance_to_goal = self.Goal_reached1(subpoint)
            while (distance_to_goal == False):
                if(steps >=step_limit_22_medium):
                    break
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
                            # FOR LEFT AS THE PREVIOUS IMPLEMENTATION
                                if self.observe_left:
                                    if y == 1:
                                        if self.distance_sensors.analogs_sensors[7] >= self.distance_sensors.right_IR_limit - 214:
                                            self.i_found_a_collision = True
                                    collision_loop.append(1)
                                    # if x!=0:
                                    # self.robot.goal_trajectory.insert(subpoint_counter+2,[0, y * 0.001,1])
                                    # else:
                                    self.robot.goal_trajectory.insert(subpoint_counter + 1, [0, y * 0.001, 1])
                            # FOR RIGHT A BIT DIFFERENT
                                else:
                                    if y == -1:
                                        if self.distance_sensors.analogs_sensors[0] >= self.distance_sensors.left_IR_limit - 214:
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
                                if self.observe_left:
                                    if y==1:
                                        if self.distance_sensors.analogs_sensors[7] >= self.distance_sensors.right_IR_limit - 214:
                                            self.i_found_a_collision = True
                                    collision_loop.append(1)
                                    # if x!=0:
                                    #    self.robot.goal_trajectory.insert(subpoint_counter+2,[0, y * 0.001,1])
                                    # else:
                                    self.robot.goal_trajectory.insert(subpoint_counter + 1, [0, y * 0.001, 1])
                                else:
                                    if y==-1:
                                        if self.distance_sensors.analogs_sensors[0] >= self.distance_sensors.left_IR_limit - 214:
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
                            if self.observe_left:
                                if y==1:
                                    if self.distance_sensors.analogs_sensors[7] >= self.distance_sensors.right_IR_limit - 214:
                                        self.i_found_a_collision = True
                                collision_loop.append(1)
                                #   if x!=0:
                                #       self.robot.goal_trajectory.insert(subpoint_counter+2,[0, y * 0.001,1])
                                #   else:
                                self.robot.goal_trajectory.insert(subpoint_counter + 1, [0, y * 0.001, 1])
                            else:
                                if y==-1:
                                    if self.distance_sensors.analogs_sensors[0] >= self.distance_sensors.left_IR_limit - 214:
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
            #print "I AM IN ROOM:",self.estimatedRooms
            # To engage the servo motor
            self.IO.servoEngage()
            self.IO.setMotors(0,0)
            self.IO.servoSet(0)
            time.sleep(1)
            self.IO.servoSet(90)
            time.sleep(1)
            self.IO.servoSet(0)
           # time.sleep(20)
            self.roomThatJustFound = self.estimatedRooms[0]
            self.estimatedRooms = []

        else:
            if self.base_found and "F" in self.estimatedRooms:
                #print "I AM IN ROOM: F"
                # To engage the servo motor
                self.IO.servoEngage()
                self.IO.setMotors(0,0)
                self.IO.servoSet(0)
                time.sleep(1)
                self.IO.servoSet(90)
                time.sleep(1)
                self.IO.servoSet(0)
               # time.sleep(20)
                self.estimatedRooms = []
                self.base_found = False
                self.roomThatJustFound = "F"
            elif self.base_found and "A" in self.estimatedRooms:
               # print "I AM IN ROOM: A"
                # To engage the servo motor
                self.IO.servoEngage()
                self.IO.setMotors(0,0)
                self.IO.servoSet(0)
                time.sleep(1)
                self.IO.servoSet(90)
                time.sleep(1)
                self.IO.servoSet(0)
            #    time.sleep(20)
                self.estimatedRooms = []
                self.base_found = False
                self.roomThatJustFound = "A"

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

    def found_box_object(self):
        self.vision.find_the_box_from_far_away = True
        time1 = time.time()
        while self.vision.find_the_box_from_far_away:
            self.IO.setMotors(0,0)
        time2 = time.time()
        print "TIME: ", time2 - time1
        print "box_far_away_found:",self.vision.box_far_away_found
        print "box_far_away_coord:",self.vision.box_far_away_coord
        move_x = self.vision.box_far_away_coord[0]/25
        move_y = self.vision.box_far_away_coord[1]/35
        if self.vision.box_far_away_found == False:
            return -999,-999

        return move_x,move_y

    def found_box_from_tsiai(self):
        self.vision.find_the_box_tsiai = True
        time1 = time.time()

        while self.vision.find_the_box_tsiai:
            self.IO.setMotors(0,0)

        time2 = time.time()

        print "TIME: ", time2 - time1
        print

        if self.vision.tsiai_found_box:
            print "distance on image x " , self.vision.box_far_away_coord_tsiai[0]
            print "distance on image y " , self.vision.box_far_away_coord_tsiai[1]
            move_x = int(self.vision.box_far_away_coord_tsiai[0]/150.0)
            move_y = int(self.vision.box_far_away_coord_tsiai[1]/150.0)

            return move_x,move_y
        return -999,-999

    def found_box_from_theo_close(self):
        self.vision.find_the_box_theo_close = True
        time1 = time.time()

        while self.vision.find_the_box_theo_close:
            self.IO.setMotors(0,0)

        time2 = time.time()

        print "TIME: ", time2 - time1
        print


        if self.vision.theo_found_box_close:
            print "distance on image x " , self.vision.box_far_away_coord_theo_close[0]
            print "distance on image y " , self.vision.box_far_away_coord_theo_close[1]
            move_x = int(self.vision.box_far_away_coord_theo_close[0]/150.0)
            move_y = int(self.vision.box_far_away_coord_theo_close[1]/150.0)

            return move_x,move_y
        return -999,-999

    def found_box_from_tsiai_super(self):
        self.vision.find_the_box_tsiai_super = True
        time1 = time.time()

        while self.vision.find_the_box_tsiai_super:
            self.IO.setMotors(0,0)

        time2 = time.time()

       # print "TIME: ", time2 - time1
       # print


        if self.vision.tsiai_found_box_super:
           # print "distance on image x " , self.vision.box_far_away_coord_tsiai_super[0]
           # print "distance on image y " , self.vision.box_far_away_coord_tsiai_super[1]
            move_x = int(self.vision.box_far_away_coord_tsiai_super[0]/150.0)
            move_y = int(self.vision.box_far_away_coord_tsiai_super[1]/150.0)

            return move_x,move_y
        return -999,-999


    def foundBoxFromTheo(self):
        self.vision.find_the_box_theo = True
        time1 = time.time()

        while self.vision.find_the_box_theo:
            pass

        time2 = time.time()

      #  print "TIME: ", time2 - time1
      #  print


        if self.vision.theo_found_box:
         #   print "distance on image x " , self.vision.box_far_away_coord_theo[0]
         #   print "distance on image y " , self.vision.box_far_away_coord_theo[1]
            move_x = int(self.vision.box_far_away_coord_theo[0]/35.0)
            move_y = int(self.vision.box_far_away_coord_theo[1])
            if move_x >=2:
                move_x = 1
            if move_x <= -2:
                move_x = -1
            return move_x,move_y
        return -999,-999

    def foundBoxFromBlackpatch(self):
        self.vision.find_the_balck_patch_tsiai = True

        while self.vision.find_the_balck_patch_tsiai:
            pass

        if self.vision.tsiai_find_balck_patch:
         #   print "distance on image x " , self.vision.box_far_away_coord_theo[0]
         #   print "distance on image y " , self.vision.box_far_away_coord_theo[1]
            move_x = int(self.vision.balck_patch_position[0]/35.0)
            move_y = int(self.vision.balck_patch_position[1])
            if move_x >=2:
                move_x = 1
            if move_x <= -2:
                move_x = -1
            return move_x,move_y
        return -999,-999
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

        longer_relative_trajectory = self.clear_180_turns(longer_relative_trajectory)

        final_relative_trajectory = self.duplicate_turns(longer_relative_trajectory)
        final_relative_trajectory = self.duplicate_turns(final_relative_trajectory)

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
            #additional_straight = int(len(line)*0.35)
            additional_straight = int(len(line)*0.15)
            line += [numpy.array([ 5.,  0.])]*additional_straight

        # merge the staight line lists with the turns
        for index, i in enumerate(indices_backup):
            straight_commands.insert(index+1+index,[commnas_list[i]])

        # flatten the final list
        final_plus_commands = [item for sublist in straight_commands for item in sublist]

        #pprint.pprint(final_plus_commands)

        return final_plus_commands

    # changes the 180 degrees turns with 4 * 45 turns
    def clear_180_turns(self,commnas_list):

        for index,i in enumerate(commnas_list):
            if i[0] == -5:
                del commnas_list[index]
                commnas_list.insert(index,numpy.array([ 0.   , -0.001]))
                commnas_list.insert(index,numpy.array([ 0.   , -0.001]))
                commnas_list.insert(index,numpy.array([ 0.   , -0.001]))
                commnas_list.insert(index,numpy.array([ 0.   , -0.001]))

        #pprint.pprint(commnas_list)
        return commnas_list

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

    # Set the desired trajectory to turn right one time
    def set_tranjectory_straight_2(self):
        traj = [[5, 0, 1],[5, 0, 1]]
        self.robot.set_desired_trajectory(traj)

    def set_tranjectory_straight_back(self):
        traj = [[-5, 0, 1],[-5, 0, 1],[-5, 0, 1],[-5, 0, 1],[-5, 0, 1],[-5, 0, 1],[-5, 0, 1]]
        self.robot.set_desired_trajectory(traj)
    def set_tranjectory_back_from_base(self):
        traj = [[-5, 0, 1],[-5, 0, 1]]
        self.robot.set_desired_trajectory(traj)
    # Set the desired trajectory to turn right one time
    def set_tranjectory_straight_6_steps(self):
        traj = [[5, 0, 1],[5, 0, 1],[5, 0, 1],[5, 0, 1],[5, 0, 1]]
        self.robot.set_desired_trajectory(traj)
    def set_tranjectory_straight_4_steps(self):
        traj = [[5, 0, 1],[5, 0, 1],[5, 0, 1]]
        self.robot.set_desired_trajectory(traj)
    def straight_traj(self):
        traj = [[5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1],
                [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1],
                [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1],
                [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1], [5, 0, 1]]


    def turn_for_box(self,move_y):
        if move_y == -1:
            traj = [[0, -0.001, 1]]
            self.robot.set_desired_trajectory(traj)
        elif move_y <= -2:
            traj = [[0, -0.001, 1],[0, -0.001, 1]]
            self.robot.set_desired_trajectory(traj)
        elif move_y == 1:
            traj = [[0, 0.001, 1]]
            self.robot.set_desired_trajectory(traj)
        elif move_y >= 2:
            traj = [[0, 0.001, 1],[0, 0.001, 1]]
            self.robot.set_desired_trajectory(traj)

    def walk_for_box(self,move_x):
        if move_x == 1:
            traj = [[5, 0, 1]]
            self.robot.set_desired_trajectory(traj)
        elif move_x == 2:
            traj = [[5, 0, 1],[5, 0, 1]]
            self.robot.set_desired_trajectory(traj)

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

    # turn robot 90 degrees after calibration
    def turn_robot_45_degrees(self,subpoint):
        steps = 0
        step_limit_90 = self.calibration.turn_360_steps/8
        while (steps < step_limit_90):
                step_point = subpoint[0:2]
                distance_to_goal = self.dynamics_control_motors(step_point, subpoint)
                steps += 1

    # turn robot 90 degrees after calibration
    def turn_robot_11_degrees(self,subpoint):
        steps = 0
        step_limit_90 = self.calibration.turn_360_steps/32
        while (steps < step_limit_90):
                step_point = subpoint[0:2]
                distance_to_goal = self.dynamics_control_motors(step_point, subpoint)
                steps += 1

    # calibration function
    def calibrate_turning_rate(self):

        calibration_data = self.turn_right_360_robot_motion_for_calibration()
        #print calibration_data
        self.calibration.calibrate_robot(calibration_data)
        print "Calibration values " ,self.calibration.turn_360_steps

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

    # turn exactly 90 degrees right
    def perform_45_degrees_turn_right(self):
        self.set_tranjectory_right()
        self.turn_robot_45_degrees(self.robot.goal_trajectory[0])
        self.IO.setMotors(0,0)

    # turn exactly 90 degrees left
    def perform_45_degrees_turn_left(self):
        self.set_tranjectory_left()
        self.turn_robot_45_degrees(self.robot.goal_trajectory[0])
        self.IO.setMotors(0,0)

     # turn exactly 90 degrees right
    def perform_11_degrees_turn_right(self):
        self.set_tranjectory_right()
        self.turn_robot_11_degrees(self.robot.goal_trajectory[0])
        self.IO.setMotors(0,0)

    # turn exactly 90 degrees left
    def perform_11_degrees_turn_left(self):
        self.set_tranjectory_left()
        self.turn_robot_11_degrees(self.robot.goal_trajectory[0])
        self.IO.setMotors(0,0)




    # Function for the
    def cage_object(self):
        self.IO.servoEngage()
        self.IO.servoSet(180)
        time.sleep(1)
        self.IO.servoSet(75)
        print "Servo in Zero!!!"
        time.sleep(1)
        self.IO.servoDisengage()
        #self.IO.servoSet(90)
        #time.sleep(1)
        #self.IO.servoSet(0)

#change open/close and light number
    def close_cage(self):
        #self.IO.servoEngage()
        self.IO.servoSet(0)
        time.sleep(3)
        #self.IO.servoDisengage()

    def open_cage(self):
        #self.IO.servoEngage()
        self.IO.servoSet(180)
        time.sleep(3)
        #self.IO.servoDisengage()

    def box_inside(self):
        light1 = self.IO.getSensors()[1]
        light2 = self.IO.getSensors()[2]
        light3 = self.IO.getSensors()[3]
        if light1 < 55 or light2 < 55 or light3 < 55:
            return True
        return False

