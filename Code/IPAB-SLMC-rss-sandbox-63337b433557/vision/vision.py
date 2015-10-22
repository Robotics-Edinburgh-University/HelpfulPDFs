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

        self.color_list = ['red','green','blue','yellow','orange','white']
        self.object_detected_list = [False,False,False,False,False]
        #setting color filter ranges
        #white filter, hsv_white = [[[30,2,255]]]
        #self.lower_white = numpy.array([0,0,50])
        #self.upper_white = numpy.array([180,20,150])
        #self.boundry_white = [self.lower_white,self.upper_white]

        #green filter, hsv_green = [[[79,255,126]]]
        self.lower_green = numpy.array([35,100,100])
        self.upper_green = numpy.array([80,255,255])
        self.boundry_green = [self.lower_green,self.upper_green]

        '''
        self.lower_black = numpy.array([0,0,0])
        self.upper_black = numpy.array([255,255,40])
        self.boundry_black = [self.lower_black,self.upper_black]
        '''
        self.lower_yellow = numpy.array([20,130,120])
        self.upper_yellow = numpy.array([35,255,255])
        self.boundry_yellow = [self.lower_yellow,self.upper_yellow]

        #red, hue 0-5
        self.lower_red = numpy.array([0,100,100])
        self.upper_red = numpy.array([10,255,255])
        self.boundry_red = [self.lower_red,self.upper_red]

        self.lower_white = numpy.array([0,0,240])
        self.upper_white = numpy.array([255,15,255])
        self.boundry_white = [self.lower_white,self.upper_white]

        self.lower_blue = numpy.array([80,100,50])
        self.upper_blue = numpy.array([130,255,255])
        self.boundry_blue = [self.lower_blue,self.upper_blue]

        self.lower_orange = numpy.array([10,150,150])
        self.upper_orange = numpy.array([20,255,255])
        self.boundry_orange = [self.lower_orange,self.upper_orange]

        #self.boundry_list = [self.boundry_yellow]
        #self.boundry_list = [self.boundry_green]
        #self.boundry_list = [self.boundry_red]
        self.boundry_list = [self.boundry_red, self.boundry_green, self.boundry_blue,self.boundry_yellow,self.boundry_orange,self.boundry_white]
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

    def ColorFilter(self,color):

        interested_contour_areas = []
        interested_contours = []
        final_contours = []
        boundry = self.boundry_list[self.color_list.index(color)]
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
            print "------------------------"
            print color, 'object detected'
            print "------------------------"
            #print "find objects"
            for contour in interested_contours:

                #cv2.drawContours(self.img, contour, -1, (0,255,0), 2)
                hull = cv2.convexHull(contour,returnPoints = False)
                #print 'contour ', interested_contours.index(contour), 'hull is', len(hull)
                defects = cv2.convexityDefects(contour,hull)
                #print 'distances'
                #print '---------------------------------------------------------'
                #print 'shape of defects', defects.shape
                total_distance = 0
                for i in range(defects.shape[0]):
                    s,e,f,d = defects[i,0]
                    distance = d/256.0
                    #print 'distance for every point ----' , distance
                    total_distance = total_distance+distance
                #print "Total distance of the hull"
                #print total_distance
                ratio_distances_volumn = distance/cv2.contourArea(contour)
                #print "distances/Volumn"
                #print ratio_distances_volumn
                if ratio_distances_volumn < 0.0015:
                    final_contours.append(contour)
                    print '========================================='
                    print "Total distance of the hull of the object"
                    print total_distance
                    print "Corresponding distances/Volumn"
                    print ratio_distances_volumn
            print '=================================================='
            print 'number of ', color, 'objects', len(final_contours)


                    #start = tuple(contour[s][0])
                    #end = tuple(contour[e][0])
                    #far = tuple(contour[f][0])
                    #cv2.line(self.img,start,end,[0,255,0],2)
                    #cv2.circle(self.img,far,5,[0,0,255],-1)
                #x,y,w,h = cv2.boundingRect(contour)
                #cv2.rectangle(self.img,(x,y),(x+w,y+h),(0,255,0),2)

        """
        if len(interested_contours) == 1:
            print color, 'object detected'
            self.object_detected_list[self.color_list.index(color)]=True
            print interested_contour_areas
        """



        #print 'interested contour size'
        #for contour in interested_contours:
        #    print cv2.contourArea(contour)

        #cv2.drawContours(self.img, contours, -1, (0,255,0),2)
        #res = cv2.bitwise_and(self.img,self.img,mask=mask)
        cv2.drawContours(self.img, final_contours, -1, (0,255,0), 2)
        cv2.drawContours(self.img, interested_contours, -1, (0,255,0), 2)
        self.IO.imshow('image',self.img)
        #return max_contour

        return len(final_contours)

    def Black_filter(self):

        interested_contour_areas = []
        interested_contours = []
        final_contours = []
        #boundry = self.boundry_list[self.color_list.index(color)]
        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        mask = cv2.inRange(gray, 15, 50)
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
            print "------------------------"
            print 'black ', 'object detected'
            print "------------------------"
            #print "find objects"
            for contour in interested_contours:

                #cv2.drawContours(self.img, contour, -1, (0,255,0), 2)
                hull = cv2.convexHull(contour,returnPoints = False)
                #print 'contour ', interested_contours.index(contour), 'hull is', len(hull)
                defects = cv2.convexityDefects(contour,hull)
                #print 'distances'
                #print '---------------------------------------------------------'
                #print 'shape of defects', defects.shape
                total_distance = 0
                for i in range(defects.shape[0]):
                    s,e,f,d = defects[i,0]
                    distance = d/256.0
                    #print 'distance for every point ----' , distance
                    total_distance = total_distance+distance
                #print "Total distance of the hull"
                #print total_distance
                ratio_distances_volumn = distance/cv2.contourArea(contour)
                #print "distances/Volumn"
                #print ratio_distances_volumn
                if ratio_distances_volumn < 0.0015:
                    final_contours.append(contour)
                    print '========================================='
                    print "Total distance of the hull of the object"
                    print total_distance
                    print "Corresponding distances/Volumn"
                    print ratio_distances_volumn
            print '=================================================='
            print 'number of ', 'black', 'objects', len(final_contours)


                    #start = tuple(contour[s][0])
                    #end = tuple(contour[e][0])
                    #far = tuple(contour[f][0])
                    #cv2.line(self.img,start,end,[0,255,0],2)
                    #cv2.circle(self.img,far,5,[0,0,255],-1)
                #x,y,w,h = cv2.boundingRect(contour)
                #cv2.rectangle(self.img,(x,y),(x+w,y+h),(0,255,0),2)

        """
        if len(interested_contours) == 1:
            print color, 'object detected'
            self.object_detected_list[self.color_list.index(color)]=True
            print interested_contour_areas
        """



        #print 'interested contour size'
        #for contour in interested_contours:
        #    print cv2.contourArea(contour)

        #cv2.drawContours(self.img, contours, -1, (0,255,0),2)
        #res = cv2.bitwise_and(self.img,self.img,mask=mask)
        cv2.drawContours(self.img, final_contours, -1, (0,255,0), 2)
        cv2.drawContours(self.img, interested_contours, -1, (0,255,0), 2)
        self.IO.imshow('image',self.img)
        return len(final_contours)

    def find_objects(self):

        objects_num_list = []
        for color in self.color_list:
            object_num = self.ColorFilter(color)
            if object_num == 0:
                objects_num_list.append(0)
            else:
                objects_num_list.append(object_num)

        #@self.Black_filter()

        return objects_num_list

        #print 'object list', object_list

        #return object_list








