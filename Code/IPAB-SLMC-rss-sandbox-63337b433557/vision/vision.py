import numpy
import numpy.linalg
import cv2
import sys
import os
import time
import math
import datetime

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
        #white filter, hsv_white = [[[30,2,255]]]
        #self.lower_white = numpy.array([0,0,50])
        #self.upper_white = numpy.array([180,20,150])
        #self.boundry_white = [self.lower_white,self.upper_white]

        #green filter, hsv_green = [[[79,255,126]]]
        self.lower_green = numpy.array([35,100,100])
        self.upper_green = numpy.array([80,255,255])
        self.boundry_green = [self.lower_green,self.upper_green]

        self.lower_yellow = numpy.array([20,130,130])
        self.upper_yellow = numpy.array([40,255,255])
        self.boundry_yellow = [self.lower_yellow,self.upper_yellow]

        #red, hue 0-5
        self.lower_red = numpy.array([0,10,10])
        self.upper_red = numpy.array([10,255,255])
        self.boundry_red = [self.lower_red,self.upper_red]

        #blue hue: light blue: 97, normal blue:85
        self.lower_blue = numpy.array([80,100,100])
        self.upper_blue = numpy.array([130,255,200])
        self.boundry_blue = [self.lower_blue,self.upper_blue]

        #self.boundry_list = [self.boundry_yellow]
        #self.boundry_list = [self.boundry_green]
        #self.boundry_list = [self.boundry_red]
        self.boundry_list = [self.boundry_red, self.boundry_green, self.boundry_blue]
        #self.boundry_list = [self.boundry_white]

    def Set_Resolution(self):
        self.IO.cameraSetResolution(self.cameraResolution)

    def ImgObtain(self):

        #read the buffer 5 times by cameraGrab and obtain it by cameraRead
        #for cleaning the buffer in case of resolution changes
        self.IO.cameraGrab()
        self.img = self.IO.cameraRead()
        #cv2.imwrite('camera-'+datetime.datetime.now().isoformat()+'.png',self.img)
        #time.sleep(1)
        #grey = cv2.cvtColor(self.img,cv2.COLOR_BGR2GRAY)
        #self.IO.imshow('grey',grey)
        #time.sleep(1)

    def ColorFilter(self,boundry):

        interested_contour_areas = []
        interested_contours = []
        hsv_image = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, boundry[0], boundry[1])
        im2,contours, hierachy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        #max_contour = contours[0]

        """
        for contour in contours:
            if cv2.contourArea(contour) > cv2.contourArea(max_contour):
                max_contour = contour
        """

        #print 'contours sizes'
        for contour in contours:
            if cv2.contourArea(contour)>100.0:
                interested_contour_areas.append(cv2.contourArea(contour))
                interested_contours.append(contour)

        
        if len(interested_contours)>0:
            #print "find objects"
            for contour in interested_contours:
                x,y,w,h = cv2.boundingRect(contour)
                cv2.rectangle(self.img,(x,y),(x+w,y+h),(0,255,0),2)


        #print 'interested contour size'
        #for contour in interested_contours:
        #    print cv2.contourArea(contour)

        #cv2.drawContours(self.img, contours, -1, (0,255,0),2)
        #res = cv2.bitwise_and(self.img,self.img,mask=mask)

        self.IO.imshow('image',self.img)
        #return max_contour

        return interested_contour_areas


    def find_objects(self):

        objects = []
        for boundry in self.boundry_list:
            object_list = self.ColorFilter(boundry)
            if len(object_list) == 0:
                objects.append([0])
            else:
                objects.append(object_list)

        return objects

        #print 'object list', object_list

        #return object_list








