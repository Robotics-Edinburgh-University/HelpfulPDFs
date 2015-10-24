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

        #control vision functionalities
        self.detect_object = False


        self.color_list = ['red','green','blue','yellow','orange']#,'white']
        #self.color_list = ['red']
        #self.object_detected_list = [0]
        self.object_detected_list = [0,0,0,0,0]

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
        self.lower_yellow = numpy.array([20,120,120])
        self.upper_yellow = numpy.array([35,255,255])
        self.boundry_yellow = [self.lower_yellow,self.upper_yellow]

        #red, hue 0-5
        self.lower_red = numpy.array([0,100,100])
        self.upper_red = numpy.array([7,255,255]) #original [10,255,255]
        self.boundry_red = [self.lower_red,self.upper_red]

        self.lower_white = numpy.array([0,0,240])
        self.upper_white = numpy.array([255,15,255])
        self.boundry_white = [self.lower_white,self.upper_white]

        self.lower_blue = numpy.array([80,100,50])
        self.upper_blue = numpy.array([130,255,255])
        self.boundry_blue = [self.lower_blue,self.upper_blue]

        self.lower_orange = numpy.array([10,130,130])
        self.upper_orange = numpy.array([20,255,255])
        self.boundry_orange = [self.lower_orange,self.upper_orange]

        self.color_list_segmentation = ['red','green','blue','yellow','orange','black']#,'white']
        self.object_detected_list_segmentation = [0,0,0,0,0,0]

        self.lower_green_segmentation = numpy.array([35,100,100])
        self.upper_green_segmentation = numpy.array([80,255,255])
        self.boundry_green_segmentation = [self.lower_green_segmentation,self.upper_green_segmentation]

        self.lower_yellow_segmentation = numpy.array([20,130,120])
        self.upper_yellow_segmentation = numpy.array([35,255,255])
        self.boundry_yellow_segmentation = [self.lower_yellow_segmentation,self.upper_yellow_segmentation]

        #red, hue 0-5
        self.lower_red_segmentation = numpy.array([0,100,100])
        self.upper_red_segmentation = numpy.array([7,255,255])
        self.boundry_red_segmentation = [self.lower_red_segmentation,self.upper_red_segmentation]

        self.lower_white_segmentation = numpy.array([0,0,240])
        self.upper_white_segmentation = numpy.array([255,15,255])
        self.boundry_white_segmentation = [self.lower_white_segmentation,self.upper_white_segmentation]

        self.lower_blue_segmentation = numpy.array([80,100,50])
        self.upper_blue_segmentation = numpy.array([130,255,255])
        self.boundry_blue_segmentation = [self.lower_blue_segmentation,self.upper_blue_segmentation]

        self.lower_orange_segmentation = numpy.array([10,150,150])
        self.upper_orange_segmentation = numpy.array([20,255,255])
        self.boundry_orange_segmentation = [self.lower_orange_segmentation,self.upper_orange_segmentation]

        self.lower_black_segmentation = numpy.array([0,0,0])
        self.upper_black_segmentation = numpy.array([255,255,80])
        self.boundry_black_segmentation = [self.lower_black_segmentation,self.upper_black_segmentation]

        self.boundry_list = [self.boundry_red, self.boundry_green, self.boundry_blue,self.boundry_yellow,self.boundry_orange]#,self.boundry_white]
        self.boundry_list_segmentation = [self.boundry_red_segmentation, self.boundry_green_segmentation, self.boundry_blue_segmentation,self.boundry_yellow_segmentation,self.boundry_orange_segmentation,self.boundry_black_segmentation]#,self.boundry_white]

    def Set_Resolution(self):
        self.IO.cameraSetResolution(self.cameraResolution)

    def ImgObtain(self):

        #read the buffer 5 times by cameraGrab and obtain it by cameraRead
        #for cleaning the buffer in case of resolution changes
        self.IO.cameraGrab()
        image = self.IO.cameraRead()
        #cv2.imwrite('camera-'+datetime.datetime.now().isoformat()+'.png',image)
        #time.sleep(1)
        return image
        #self.img = self.IO.cameraRead()

    def Blur(self,image,kernel_sz):
        kernel_size = (kernel_sz,kernel_sz)
        kernel = numpy.ones(kernel_size,numpy.float32)
        kernel = kernel/kernel.sum()
        average_img = cv2.filter2D(image,-1,kernel)

        return average_img


    def HSV_Conversion(self,image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        return hsv_image


    def ColorFilter(self,color,origin_image,hsv_image):

        interested_contour_areas = []
        interested_contours = []
        final_contours = []
        boundry = self.boundry_list[self.color_list.index(color)]
        #hsv_image = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, boundry[0], boundry[1])
        im2,contours, hierachy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

        for contour in contours:
            if cv2.contourArea(contour)>200.0:
                interested_contour_areas.append(cv2.contourArea(contour))
                interested_contours.append(contour)


        if len(interested_contours)>0:
            print "------------------------"
            print color, 'object detected'
            print "------------------------"
            print "find objects"
            for contour in interested_contours:

                #cv2.drawContours(self.img, contour, -1, (0,255,0), 2)
                hull = cv2.convexHull(contour,returnPoints = False)
                #print 'contour ', interested_contours.index(contour), 'hull is', len(hull)
                defects = cv2.convexityDefects(contour,hull)
                total_distance = 0
                for i in range(defects.shape[0]):
                    s,e,f,d = defects[i,0]
                    distance = d/256.0
                    total_distance = total_distance+distance
                #print "Total distance of the hull"
                #print total_distance
                ratio_distances_volumn = distance/cv2.contourArea(contour)
                print "distances/Volumn"
                print ratio_distances_volumn
                if ratio_distances_volumn < 0.0025:
                    final_contours.append(contour)
                    print '========================================='
                    print "Total distance of the hull of the object"
                    print total_distance
                    print "Corresponding distances/Volumn"
                    print ratio_distances_volumn
            print '=================================================='
            print 'number of ', color, 'objects', len(final_contours)

        #print 'interested contour size'
        #for contour in interested_contours:
        #    print cv2.contourArea(contour)

        #cv2.drawContours(self.img, contours, -1, (0,255,0),2)
        #res = cv2.bitwise_and(self.img,self.img,mask=mask)
        cv2.drawContours(origin_image, interested_contours, -1, (0,255,0), 2)
        cv2.drawContours(origin_image, final_contours, -1, (0,255,0), 2)
        self.IO.imshow('image',origin_image)
        #return max_contour

        return len(final_contours)


    def find_objects(self,img):

        # if controller commands to detect objects detect
        if self.detect_object:
            objects_num_list = []
            blur_image = self.Blur(img,5)
            hsv_image = self.HSV_Conversion(blur_image)
            for color in self.color_list:
                object_num = self.ColorFilter(color,img,hsv_image)
                if object_num == 0:
                    objects_num_list.append(0)
                else:
                    objects_num_list.append(object_num)

            #@self.Black_filter()
            self.object_detected_list = objects_num_list
            self.detect_object = False

        #print 'object list', object_list

        #return object_list

    def Segmentation_RGBXY(self,img,cluster_numbers):
        time1 = time.time()
        #Clastering number,setting 5

        # Construct an array of 3D points in the RGB colour space.
        # Each pixel in the image is represented as a single point.
        rgb_pts = numpy.float32(img.reshape(-1, 3))

        # Create an array of 2D points corresponding to the xy position of each pixel in the image
        xy_pts = numpy.indices(img.shape[:2]).reshape(2, -1).transpose()

        # Since colour and position have different units we should weigh them appropriately
        # such that both are equally important during clustering. The value of 0.2 is
        # empirically determined, but you should try other values e.g. 0.1, 0.5, 1.0
        # If the data is first normalised this step will not be necessary.
        xy_weight = 0.2
        # Stack the 3D RGB points and the 2D xy points into 5D points
        rgbxy_pts = numpy.float32(numpy.hstack((rgb_pts, xy_weight * xy_pts)))

        # Create the criteria when the K-means iterations should be terminated.
        # The iteration will stop if the error is less then EPS = 1.0 or the
        # maximum number of iteration MAX_ITER = 20 is achieved
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)

        # Run K-means. The return values are:
        # ret     - the final value of the minimised sum of distances
        # labels  - the cluster which each point belons to
        # centers - the centers of the K clusters
        ret,labels,centers=cv2.kmeans(rgbxy_pts,  # Input points
                              cluster_numbers,          # Number of clusters
                              None,       # Initial estimate of the label of each point
                              criteria,   # Termination criteria
                              10,         # Return the best result after that many attempts
                              cv2.KMEANS_RANDOM_CENTERS)  # Initialise the clusters randomly

        # Convert the centers from float32 to uint8 and take only the RGB coordinates
        centers = numpy.uint8(centers[:, :3])

        # Set the coordinates of each point to the center of the cluster it belongs to
        seg_rgbxy_pts = centers[labels.flatten()]

        # Make the RGB points array into an image again
        seg_rgbxy_img = seg_rgbxy_pts.reshape((img.shape))
        time2 = time.time()

        #print "segmentation time ", time2 - time1
        self.IO.imshow('image',seg_rgbxy_img)

        return seg_rgbxy_img


    def ColorFilter_segmentation(self,color,origin_image,seg_image_hsv):

        interested_contour_areas = []
        interested_contours = []
        final_contours = []

        boundry = self.boundry_list_segmentation[self.color_list_segmentation.index(color)]
        #hsv_image_segmented = cv2.cvtColor(seg_rgbxy_img, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(seg_image_hsv, boundry[0], boundry[1])
        im2,contours, hierachy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

        for contour in contours:
            if cv2.contourArea(contour)>200.0:
                interested_contour_areas.append(cv2.contourArea(contour))
                interested_contours.append(contour)

        #cv2.drawContours(self.img, contours, -1, (0,255,0),2)
        #res = cv2.bitwise_and(self.img,self.img,mask=mask)
        #cv2.drawContours(origin_image, final_contours, -1, (0,255,0), 2)
        #cv2.drawContours(origin_image, interested_contours, -1, (0,255,0), 2)
        #self.IO.imshow('image',origin_image)
        #return max_contour

        if len(interested_contours)>0:
            print "------------------------"
            print color, 'object detected'
            print "------------------------"
            print "find objects"
            print "number of ", color, "objetcs fonnd: "
            print len(interested_contours)

        return len(interested_contours)


    def find_objects_segmentation(self,img):

        # if controller commands to detect objects detect
        if self.detect_object:
            #time1 = time.time()
            objects_num_list = []
            blur_image = self.Blur(img,5)
            segmented_image = self.Segmentation_RGBXY(blur_image,cluster_numbers = 6)
            hsv_image_seg = self.HSV_Conversion(segmented_image)
            for color in self.color_list_segmentation:
                object_num = self.ColorFilter_segmentation(color,img,hsv_image_seg)
                if object_num == 0:
                    objects_num_list.append(0)
                else:
                    objects_num_list.append(object_num)

            #@self.Black_filter()
            self.object_detected_list = objects_num_list
            self.detect_object = False
            #time2 = time.time()
            #print "time interval", time2-time1

        #print 'object list', object_list

        #return object_list








