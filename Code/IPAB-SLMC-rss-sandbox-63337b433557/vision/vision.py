import numpy
import numpy.linalg
import cv2
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

class robot_vision:

    def __init__(self,io):

        #IO
        self.IO = io

        #resolution
        self.cameraResolution = 'low'

        #parameters
        self.hasImage = False
        self.res = 0
        self.sw = False
        self.swPrew = False

        #read an image and creat the array stores images
        self.IO.cameraGrab()
        self.img = self.IO.cameraRead()

        #setting color filter ranges
        self.lower_yellow = numpy.array([0,100,100])
        self.upper_yellow = numpy.array([5,255,255])

    def Set_Resolution(self):
        self.IO.cameraSetResolution(self.cameraResolution)

    def ImgObtain(self):

        #read the buffer 5 times by cameraGrab and obtain it by cameraRead
        #for cleaning the buffer in case of resolution changes
            self.IO.cameraGrab()
            self.img = self.IO.cameraRead()

        #self.IO.imshow('window',self.img)
        #time.sleep(1)

    def ColorFilter(self):

        hsv_image = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, self.lower_yellow, self.upper_yellow)
        res = cv2.bitwise_and(self.img,self.img,mask=mask)

        self.IO.imshow('filter',res)








