#!/usr/bin/env python
__TODDLER_VERSION__="1.0.0"

import time
import numpy
import cv2
import datetime
import os
import sys
import signal
import math


# Get path of to the toddler file... to use always relative paths
CURRENT_PATH = os.getcwd() 


# import robot status modules
sys.path.insert(0, CURRENT_PATH + '/robot')
from robot_manager import Robot_manager

# import vision status modules
sys.path.insert(0, CURRENT_PATH + '/vision')
from vision import robot_vision


# Hardware test code
class Toddler:
    def __init__(self,IO):
        print 'I am a toddler playing in a sandbox'
        self.IO=IO
        self.inp=[0, 0, 0, 0, 0, 0, 0, 0]
        self.robot_manager = Robot_manager(self.IO)
        self.RobotVision = robot_vision(self.IO)


    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Control(self, OK):
	
	# Move forward and backward in collision
        # self.buttonSensor.move_back_due_collision()
	
	# test the light sensors
	#while (1):	
	 #   self.lightSensor.measure_values_on_axis()#calculate_light_sensor_difference()
	 #   time.sleep(0.1)
	
	#while (1):
	  #self.sensors.update_analog_sensors_meas()
	  #print self.sensors.analogs_sensors
	  #time.sleep(0.1)
	"""
	while (1):
	    start = time.time()
	    self.robot_manager.move_robot_to_goal_aris()
	    #print self.robot_manager.overall_sensors_direction()
            self.IO.setMotors(0,0)
	    end = time.time()
	    print "time for 16 steps" , end-start
	    print "time per steps" , (end-start)/16 
	    print " over \n"
	    time.sleep(5)
	"""
	
	
	
    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Vision(self, OK):

        #color_list = ['red','green','blue']
        #Object_detected_list = [False, False, False]
        while(OK):
            self.RobotVision.Set_Resolution()
            self.RobotVision.ImgObtain()
            detection = self.RobotVision.find_objects()
            #counter = 0
            #print detection
            #print "length of object list"
            #print len(detection)
            #for element in detection:

                #print 'area'
                #if len(element)>0:
                #print 'color is ', color_list[counter]
                #if len(element) == 1:
                    #print self.RobotVision.color_list[counter],'object detected'
                    #self.RobotVision.Object_detected_list[counter] = True
                #print element
               # counter = counter + 1

            time.sleep(0.5)

        """
        self.IO.cameraSetResolution('low')
        hasImage=False
        res=0
        sw=False
        swPrev=False
        while OK():
            if self.inp[4]:
                for i in range(0,5):
                    self.IO.cameraGrab()
                img=self.IO.cameraRead()
                if img.__class__==numpy.ndarray:
                    hasImage=True
                    cv2.imwrite('camera-'+datetime.datetime.now().isoformat()+'.png',img)
                    self.IO.imshow('window',img)
                    self.IO.setStatus('flash',cnt=2)
                    time.sleep(0.5)
            if hasImage:    
                self.IO.imshow('window',img)
                
            sw=self.inp[5]
            if sw!=swPrev and sw:
                res=(res+1)%4
                if res==0:
                    self.IO.cameraSetResolution('low')
                    self.IO.setError('flash',cnt=1)
                if res==1:
                    self.IO.cameraSetResolution('medium')
                    self.IO.setError('flash',cnt=2)
                if res==2:
                    self.IO.cameraSetResolution('high')
                    self.IO.setError('flash',cnt=3)
                if res==3:
                    self.IO.cameraSetResolution('full')
                    self.IO.setError('flash',cnt=4)
                time.sleep(0.5)
            swPrev=sw
            
            time.sleep(0.05)
            """

    def stop_motors_on_interupt(self):
        print('You pressed Ctrl+C!')
        time.sleep(1)
        self.IO.setMotors(0,0)
        time.sleep(1)
        self.IO.setMotors(0,0)


