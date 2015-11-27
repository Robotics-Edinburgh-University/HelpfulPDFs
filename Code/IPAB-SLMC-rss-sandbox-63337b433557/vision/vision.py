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
        #self.cameraResolution = 'high'

        #parameters
        self.hasImage = False
        self.res = 0
        self.sw = False
        self.swPrew = False

        #read an image and creat the array stores images
        self.IO.cameraGrab()
        self.img = self.IO.cameraRead()

        #control vision functionalities
        self.detect_object = True

        self.find_the_box_from_far_away = False
        self.box_far_away_found = False
        self.box_far_away_coord = [0,0]
        self.find_the_box_tsiai = False
        self.find_the_box_theo = False

        self.tsiai_found_box = False
        self.theo_found_box = False

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

        self.color_list_segmentation = ['red','green','blue','yellow','orange']#,'white']#,'black']#,'white']
        self.object_detected_list_segmentation = [0,0,0,0,0]

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

        self.lower_white_segmentation = numpy.array([0,0,180])
        self.upper_white_segmentation = numpy.array([255,255,255])
        self.boundry_white_segmentation = [self.lower_white_segmentation,self.upper_white_segmentation]

        self.lower_blue_segmentation = numpy.array([85,100,50])
        #wrong
        #self.lower_blue_segmentation = numpy.array([80,100,50])
        self.upper_blue_segmentation = numpy.array([130,255,255])
        self.boundry_blue_segmentation = [self.lower_blue_segmentation,self.upper_blue_segmentation]

        self.lower_orange_segmentation = numpy.array([10,150,150])
        self.upper_orange_segmentation = numpy.array([20,255,255])
        self.boundry_orange_segmentation = [self.lower_orange_segmentation,self.upper_orange_segmentation]

        self.lower_black_segmentation = numpy.array([0,0,0])
        self.upper_black_segmentation = numpy.array([255,255,80])
        self.boundry_black_segmentation = [self.lower_black_segmentation,self.upper_black_segmentation]

        self.lower_white_rgb = numpy.array([200,200,200])
        self.upper_white_rgb = numpy.array([255,255,255])

        self.boundry_list = [self.boundry_red, self.boundry_green, self.boundry_blue,self.boundry_yellow,self.boundry_orange]#,self.boundry_white]
        self.boundry_list_segmentation = [self.boundry_red_segmentation, self.boundry_green_segmentation, self.boundry_blue_segmentation,self.boundry_yellow_segmentation,self.boundry_orange_segmentation]#,self.boundry_white_segmentation]#,self.boundry_black_segmentation]#,self.boundry_white]

        self.box_from_close_algorithm = False

        self.Mario_origin = cv2.imread('./vision/mario.png',0)
        self.Mario_feature_pts = cv2.imread('./vision/mario_feature_points.png')
        self.Mario_template_close = cv2.imread('./vision/mario_close.png',0)
        self.Mario_template_mid = cv2.imread('./vision/mario_mid.png',0)
        self.Mario_template_far = cv2.imread('./vision/mario_resize_60.png',0)
        self.Mario = [self.Mario_template_close,self.Mario_template_mid,self.Mario_template_far]
        #self.Mario_thre = [0.55,0.55,0.57]#far threshold higher in case of wrong detection of walls
        self.Mario_thre = [0.5,0.5,0.5]

        self.Wario_origin = cv2.imread('./vision/wario.png',0)
        self.Wario_feature_pts = cv2.imread('./vision/wario_feature_points.png')
        self.Wario_template_close = cv2.imread('./vision/wario_close.png',0)
        self.Wario_template_mid = cv2.imread('./vision/wario_mid.png',0)
        self.Wario_template_far = cv2.imread('./vision/wario_resize_60.png',0)
        self.Wario = [self.Wario_template_close,self.Wario_template_mid,self.Wario_template_far]
        #self.Wario_thre = [0.55,0.55,0.57]#far threshold higher in case of wrong detection of walls
        self.Wario_thre = [0.5,0.5,0.5]

        self.Zoidberg_origin = cv2.imread('./vision/zoidberg.png',0)
        self.Zoidberg_feature_pts = cv2.imread('./vision/zoidberg_feature_points.png')
        self.Zoidberg_template_close = cv2.imread('./vision/zoidberg_close.png',0)
        self.Zoidberg_template_mid = cv2.imread('./vision/zoidberg_mid.png',0)
        self.Zoidberg_template_far = cv2.imread('./vision/zoidberg_resize_60.png',0)
        self.Zoidberg = [self.Zoidberg_template_close,self.Zoidberg_template_mid,self.Zoidberg_template_far]
        #self.Zoidberg_thre = [0.55,0.55,0.59]#far threshold higher in case of wrong detection of walls
        self.Zoidberg_thre = [0.5,0.5,0.5]

        self.Watching_origin = cv2.imread('./vision/watching.png',0)
        self.Watching_feature_pts = cv2.imread('./vision/watching_feature_points.png')
        self.Watching_template_close = cv2.imread('./vision/watching_close.png',0)
        self.Watching_template_mid = cv2.imread('./vision/watching_mid.png',0)
        self.Watching_template_far = cv2.imread('./vision/watching_resize_60.png',0)
        self.Watching = [self.Watching_template_close,self.Watching_template_mid,self.Watching_template_far]
        #self.Watching_thre = [0.55,0.55,0.57]#far threshold higher in case of wrong detection of walls
        self.Watching_thre = [0.5,0.515,0.51]

        self.template_names = ['Mario','Wario','Zoidberg','Watching']
        self.feature_pts_template = [self.Mario_feature_pts,self.Wario_feature_pts,self.Zoidberg_feature_pts,self.Watching_feature_pts]
        self.template_origins = [self.Mario_origin,self.Wario_origin,self.Zoidberg_origin,self.Watching_origin]
        self.distance = ['close','mid','far']
        self.distance_range = [[481,554],[353,461],[249,328]]

    def Set_Resolution(self,res):
        self.IO.cameraSetResolution(res)

    def show_template(self,template):
        #print len(self.Mario_template)
        #for template in templates:
        self.IO.imshow('template',template)
        #time.sleep(2)
    def ImgObtain(self):

        #read the buffer 5 times by cameraGrab and obtain it by cameraRead
        #for cleaning the buffer in case of resolution changes
        self.IO.cameraGrab()
        image = self.IO.cameraRead()
        #cv2.imwrite('camera-'+datetime.datetime.now().isoformat()+'.png',image)
        #self.IO.imshow('image',image)
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

    def White_Filter_BGR(self,origin_img,img):

        interested_contour_areas = []
        interested_contours = []
        mask = cv2.inRange(img, self.lower_white_rgb, self.upper_white_rgb)
        im2,contours, hierachy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        for contour in contours:
            #cv2.drawContours(origin_img, contour, -1, (0,255,0), 2)
            if ((cv2.contourArea(contour)>20.0) and (cv2.contourArea(contour)<250.0)):
                interested_contour_areas.append(cv2.contourArea(contour))
                interested_contours.append(contour)

        print "area of interested white objects"
        print interested_contour_areas

        cv2.drawContours(origin_img, interested_contours, -1, (0,255,0), 2)
        ###self.IO.imshow('image',origin_img)
        #cv2.imwrite('camera-'+datetime.datetime.now().isoformat()+'white_filter'+'.png',origin_image)

        if len(interested_contours)>2:
            print "------------------------"
            print len(interested_contours), "white objects detected"
            white_obj_num = 1
        else:
            white_obj_num = 0

        return white_obj_num


    def Draw_contours(self,origin_image,contours):

        cv2.drawContours(origin_image, contours, -1, (0,255,0), 2)
        ###self.IO.imshow('image',origin_image)

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
                ratio_distances_volumn = total_distance/cv2.contourArea(contour)
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
        ###self.IO.imshow('image',origin_image)

        return len(final_contours)


    def find_objects(self,img):

        # if controller commands to detect objects detect
        if self.detect_object:
            objects_num_list = []
            blur_image = self.Blur(img,5)
            white_object = self.White_Filter_BGR(img,blur_image)
            hsv_image = self.HSV_Conversion(blur_image)
            for color in self.color_list:
                object_num = self.ColorFilter(color,img,hsv_image)
                if object_num == 0:
                    objects_num_list.append(0)
                else:
                    objects_num_list.append(object_num)
            objects_num_list.append(white_object)

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
        #self.IO.imshow('image',seg_rgbxy_img)

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
        cv2.drawContours(origin_image, interested_contours, -1, (0,255,0), 2)
        ###self.IO.imshow('image',origin_image)
        #cv2.imwrite('camera-'+datetime.datetime.now().isoformat()+'white_filter'+'.png',origin_image)
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
        self.detect_object = True
        if self.detect_object:
            #self.IO.imshow('image',img)
            #time1 = time.time()
            objects_num_list = []
            white_object_num = self.White_Filter_BGR(img,img)
            blur_image = self.Blur(img,5)
            segmented_image = self.Segmentation_RGBXY(blur_image,cluster_numbers = 6)
            #white_object_num = self.White_Filter_BGR(img,segmented_image)
            hsv_image_seg = self.HSV_Conversion(segmented_image)
            for color in self.color_list_segmentation:
                object_num = self.ColorFilter_segmentation(color,img,hsv_image_seg)
                if object_num == 0:
                    objects_num_list.append(0)
                else:
                    objects_num_list.append(object_num)
            objects_num_list.append(white_object_num)

            #@self.Black_filter()
            self.object_detected_list = objects_num_list
            self.detect_object = False
            #time2 = time.time()
            #print "time interval", time2-time1

        #print 'object list', object_list

        #return object_list


    def Lock_Cubes(self,templates,thresholds):#,origin_img):
        if self.find_the_box_tsiai:
            self.Set_Resolution('high')
            origin_img = self.ImgObtain()
            #cv2.imwrite('camera-'+datetime.datetime.now().isoformat()+'.png',origin_img)
            #self.IO.imshow('img',origin_img)
            origin_gray_img = cv2.cvtColor(origin_img,cv2.COLOR_BGR2GRAY)
            methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR',
                        'cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']
            method = eval('cv2.TM_CCOEFF_NORMED')
            found = 0
            #print "sssssssssss in sides"

            for index_threshold , template in enumerate(templates):
                w, h = template.shape[::-1]
                result = cv2.matchTemplate(origin_gray_img,template,method)
                min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
                print "max value of",self.distance[index_threshold], max_val

            #loc = numpy.where( result >= 0.55)
            #print len(loc)
            #for pt in zip(*loc[::-1]):
                top_left = max_loc
                bottom_right = (top_left[0] + w, top_left[1] + h)
                center = numpy.array([400,500])
                if max_val > thresholds[index_threshold]:
                    distance_range = self.distance_range[index_threshold]   #object should stay in this y distance range
                    obj_center = numpy.array([top_left[0]+w/2,top_left[1]+h/2])
                    cv2.rectangle(origin_img,top_left, bottom_right, (0,0,255), 3)

                    if obj_center[1]>distance_range[0] and obj_center[1]<distance_range[1]:
                        cv2.rectangle(origin_img,top_left, bottom_right, (0,255,0), 3)
                        self.box_far_away_coord_tsiai = center - obj_center
                        found = 1
                        print "found box in<-----",self.distance[index_threshold]
                    #print "center:", center
                    #cv2.circle(origin_img,(obj_center[0],obj_center[1]), 3, (0,0,255), 2)
                        if index_threshold == 0:
                            self.box_from_close_algorithm = True
                        else:
                            self.box_from_close_algorithm = False

                if found:
                    self.tsiai_found_box = True
                    self.find_the_box_tsiai = False
                    break
                else:
                    self.tsiai_found_box = False
                    self.box_far_away_coord_tsiai = numpy.array([0,0])
            self.find_the_box_tsiai = False
            #self.IO.imshow('img',origin_img)
            self.Set_Resolution('low')

    def Blue_filter(self):
        self.Set_Resolution('high')
        origin_img = self.ImgObtain()
        hsv_image = self.HSV_Conversion(origin_img)
        lower_blue_segmentation = numpy.array([85,100,50])
        upper_blue_segmentation = numpy.array([130,255,255])
        mask_blue = cv2.inRange(hsv_image, lower_blue_segmentation, upper_blue_segmentation)
        im2,contours, hierachy = cv2.findContours(mask_blue,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

        for cnts in contours:
            x,y,w,h = cv2.boundingRect(cnts)
            cv2.rectangle(origin_img,(x,y),(x+w,y+h),(0,255,0),2)
        self.IO.imshow('img',origin_img)
        self.Set_Resolution('low')




    def Check_Object(self,temp_name):

        MIN_MATCH_COUNT = 10

        index = self.template_names.index(temp_name)
        template = self.feature_pts_template[index]
        self.Set_Resolution('high')
        origin_img = self.ImgObtain()
        img = self.Blur(origin_img,kernel_sz= 3)
        #img = origin_img
        #create ORB detector
        orb = cv2.ORB_create(nfeatures = 1000)
        # Create a brute force matcher object.
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, # Tells the matcher how to compare descriptors.
                                     # We have set it to use the hamming distance between
                                     # descriptors as ORB provides binary descriptors.
                        crossCheck=True)  # Enable cross check. This means that the matcher
                                     # will return a matching pair [A, B] iff A is the
                                     # closest point to B and B is the closest to A.
        # Detect the feature points in the object and the scene images
        template_kp, template_des = orb.detectAndCompute(template, None)
        img_kp, img_des = orb.detectAndCompute(img, None)

        # Find the matching points using brute force
        matches = bf.match(img_des, template_des)

        # Sort the matches in the order of the distance between the
        # matched descriptors. Shorter distance means better match.
        matches = sorted(matches, key = lambda m: m.distance)

        #find good matches:
        good = []
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)

        if len(good)>MIN_MATCH_COUNT:
            src_pts = numpy.float32([ template_kp[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
        else:
            print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
            matchesMask = None


        # Use only the best 1/10th of matchess
        matches = matches[:len(matches)/10]

        # Visualise the detected matching pairs
        match_vis = cv2.drawMatches(img,       # First processed image
                                    img_kp,    # Keypoints in the first image
                                    template,         # Second processed image
                                    template_kp,      # Keypoints in the second image
                                    matches,     # Detected matches
                                    None)        # Optional output image

        # Create numpy arrays with the feature points in order to
        # estimate the homography between the two images.
        img_pts = numpy.float32([img_kp[m.queryIdx].pt for m in matches])
        template_pts = numpy.float32([template_kp[m.trainIdx].pt for m in matches])

        # Calculate the homography between the two images. The function works
        # by optimising the projection error between the two sets of points.
        H, masked_pts = cv2.findHomography(template_pts,    # Source image
                                           img_pts,  # Destination image
                                           cv2.RANSAC) # Use RANSAC because it is very likely to have wrong matches

        #calculate the inlier ratio
        m_pts = [float(item) for sublist in masked_pts for item in sublist]
        inlier_ratio = sum(m_pts)/len(m_pts)

        print "Feature points in the bounding box", inlier_ratio

        # Get the size of the object image
        h, w = template.shape[:2]
        # Create an array with points at the 4 corners of the image
        bounds = numpy.float32([[[0, 0], [w, 0], [w, h], [0, h]]])

        # Project the object image corners in the scene image
        # in order to find the object
        bounds = cv2.perspectiveTransform(bounds, H).reshape(-1, 2)

        #  Highlight the detected object
        for i in range(4):
            # Draw the sides of the box by connecting consecutive points
            # The line point index is i. Get the index of the second point
            j = (i + 1) % 4
            cv2.line(match_vis,                    # Image where to draw
                    (bounds[i][0], bounds[i][1]), # First point of the line
                    (bounds[j][0], bounds[j][1]), # Second point of the line
                    (255, 255, 255),              # Colour (B, G, R)
                    3)                            # Line width in pixels

        # Draw a circle at each projected corner
        match_vis = cv2.circle(match_vis, (bounds[0][0], bounds[0][1]), 10, (255, 0, 0), -1)
        match_vis = cv2.circle(match_vis, (bounds[1][0], bounds[1][1]), 10, (0, 255, 0), -1)
        match_vis = cv2.circle(match_vis, (bounds[2][0], bounds[2][1]), 10, (0, 0, 255), -1)
        match_vis = cv2.circle(match_vis, (bounds[3][0], bounds[3][1]), 10, (0, 255, 255), -1)

        # Show the object detection
        self.IO.imshow('img',match_vis)

        self.Set_Resolution('low')



    def Cube_Detection(self,origin_img):
        time1 = time.time()
        origin_gray = cv2.cvtColor(origin_img,cv2.COLOR_BGR2GRAY)
        gray_gaussian = cv2.GaussianBlur(origin_gray,(7,7),0)
        canny_edge = cv2.Canny(gray_gaussian,120,250)
        #self.IO.imshow('img',canny_edge)
        #construct and apply a closing kernel to 'close' gaps between 'white' pixels
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(15,15))
        closed =cv2.morphologyEx(canny_edge,cv2.MORPH_CLOSE,kernel)

        self.IO.imshow('img',closed)

        im2,cnts,hierarchky = cv2.findContours(closed.copy(),cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #im2,cnts,hierarchky = cv2.findContours(canny_edge.copy(),cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        total = 0

        for c in cnts:
            if cv2.contourArea(c)>150 and cv2.contourArea(c)<10000 :
                #x,y,w,h = cv2.boundingRect(c)
                #if  w*h > 2000  and w*h < 6000:
                peri = cv2.arcLength(c,True)
                approx = cv2.approxPolyDP(c,0.02*peri,True)
                x,y,w,h = cv2.boundingRect(approx)
                if len(approx) > 3 and len(approx)<11:
                    if w*h<10000:
                        cv2.drawContours(origin_img,[approx],-1,(0,255,0),4)
                        total += 1
                        print "number of convex",len(approx)
                        print "original controu area", cv2.contourArea(c)
                #print "contour area", w*h


        time2 = time.time()
        print "time_diff", time2 - time1
        print "------------------------"
        self.IO.imshow('img',origin_img)





    def detect_resources(self,img):

        if self.find_the_box_from_far_away:
            xy_r = 40 # The spatial window radius
            rgb_r = 40 # The color window radius

            #Run meanshift
            meanshift_img = cv2.pyrMeanShiftFiltering(img, xy_r, rgb_r)

            #cv2.imwrite('camera-'+datetime.datetime.now().isoformat()+'.png',img)

            # Convert image to Hsv
            hsv = cv2.cvtColor(meanshift_img,cv2.COLOR_BGR2HSV)

            #wall 23, 48,148
            wall_lower = numpy.array([10, 35, 130])
            wall_upper = numpy.array([30, 60, 160])

            #floor 103,63,183
            floor_lower = numpy.array([90, 50, 170])
            floor_upper = numpy.array([115, 80, 190])

            # find the colors within the specified boundaries and apply
            # the mask
            mask_wall = cv2.inRange(hsv, wall_lower, wall_upper)
            mask_floor = cv2.inRange(hsv, floor_lower, floor_upper)

            lower_green_segmentation = numpy.array([35,100,100])
            upper_green_segmentation = numpy.array([80,255,255])
            mask_green = cv2.inRange(hsv, lower_green_segmentation, upper_green_segmentation)

            lower_yellow_segmentation = numpy.array([20,130,120])
            upper_yellow_segmentation = numpy.array([35,255,255])
            mask_yellow = cv2.inRange(hsv, lower_yellow_segmentation, upper_yellow_segmentation)

            #red, hue 0-5
            lower_red_segmentation = numpy.array([0,100,100])
            upper_red_segmentation = numpy.array([7,255,255])
            mask_red = cv2.inRange(hsv, lower_red_segmentation, upper_red_segmentation)

            lower_white_segmentation = numpy.array([0,0,180])
            upper_white_segmentation = numpy.array([255,255,255])
            mask_white = cv2.inRange(hsv, lower_white_segmentation, upper_white_segmentation)

            lower_blue_segmentation = numpy.array([85,100,50])
            upper_blue_segmentation = numpy.array([130,255,255])
            mask_blue = cv2.inRange(hsv, lower_blue_segmentation, upper_blue_segmentation)

            lower_orange_segmentation = numpy.array([10,150,150])
            upper_orange_segmentation = numpy.array([20,255,255])
            mask_orange = cv2.inRange(hsv, lower_orange_segmentation, upper_orange_segmentation)

            mask_colors = mask_green + mask_yellow + mask_green + mask_orange + mask_blue + mask_red #+ mask_wall + mask_floor
            mask_colors_inv = cv2.bitwise_not(mask_colors)
            #output = cv2.bitwise_and(hsv, hsv, mask = mask_colors_inv)

            im2, contours, hierarchy = cv2.findContours(mask_colors,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 1:
                contour_lens = [len(i) for i in contours]
                #print contour_lens
                max_contour_index = contour_lens.index(max(contour_lens))

                (x,y),radius = cv2.minEnclosingCircle(contours[max_contour_index])
                center = (int(x),int(y))
                radius = 2*int(radius)
                img = cv2.circle(img,center,radius,(0,255,0),2)
                obstacle_circle = circle(center[0],center[1],radius)
                 #if len(contour) > 10:
            else:
                obstacle_circle = circle(0,0,0.1)

            #final = cv2.cvtColor(output,cv2.COLOR_HSV2BGR)
            ######
            # till here filter and noisy object detection....
            ######

            #Convert the scene image to a grayscale image
            gray = cv2.cvtColor(meanshift_img, cv2.COLOR_BGR2GRAY)
            #Run the Harris corner detector
            harris_response = cv2.cornerHarris(gray, # Image to be procesed
                                                   2, # Size of the window (2x2)
                                                   3, # Size of the Sobel filter used to calculate
                                                   # the gradients. Can be 1, 3, 5 or 7.
                                                   0.07)   # Value for the free prameter k
            #Calculate a threshold for the corner response
            thresh = harris_response.max() * 0.1
            #Find all pixels which have harris detector response greater than the threshold
            corners = numpy.where(harris_response > thresh)

            if len(corners[0]) == 0 :
               #print "Nothing was found !!!"
               #print "NUmber of corners ", len(corners[0])
               self.box_far_away_found = False
               self.box_far_away_coord = [0,0]
               self.find_the_box_from_far_away = False
               return
#               return False, numpy.array([0,0])
            else:
                #Draw a circle centred at each detected corner
                points_interest = []
                for x, y in zip(corners[1], corners[0]):
                    p_i = circle(x,y,1)
                    if self.is_overlap(obstacle_circle,p_i) == False:
                        points_interest.append((x,y))
                        cv2.circle(meanshift_img,   # Image where to draw
                               (x, y),          # Centre of the circle
                               1,               # Radius of the circle
                               (0, 255, 0),     # Colour (B, G, R)
                               1)               # Line width

                self.IO.imshow('cornors',meanshift_img)
                if len(points_interest) > 0:
                    p_is = numpy.array(points_interest , dtype=numpy.int)
                    area = cv2.contourArea(p_is)
                    x,y,w,h = cv2.boundingRect(p_is)
                    rect_area = w*h
                    print "area " , area , rect_area
                    image_center = numpy.array( [80 , 100] )
                    if area < 100 and rect_area < 300 :
                        print "object found !!! "
                        obj_coord = numpy.array([ int(numpy.mean(p_is[:,0])) , int(numpy.mean(p_is[:,0])) ] )
                        self.box_far_away_found = True
                        self.box_far_away_coord = image_center - obj_coord
                        self.find_the_box_from_far_away = False
                        return
                        #return True, image_center - obj_coord
                    else:
                        #print " too many corners detected "
                        self.box_far_away_found = False
                        self.box_far_away_coord = numpy.array([0,0])
                        self.find_the_box_from_far_away = False
                        return
#                       return  False, numpy.array([0,0])
                else:
                    #print " No interesting points"
                    self.box_far_away_found = False
                    self.box_far_away_coord = numpy.array([0,0])
                    self.find_the_box_from_far_away = False
                    return
#                    return  False, numpy.array([0,0])
                #x,y,w,h = cv2.boundingRect(p_is)
                #rect_area = w*h
                #print "rect_area ", rect_area


    def is_overlap(self,circle1, circle2):
        distance = ((circle1.x - circle2.x)**2 + (circle1.y - circle2.y)**2)**0.5
        return distance < circle1.r + circle2.r


    def detect_resources_new_version(self,img):

        if self.find_the_box_theo:
            found = 0

            cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))

            kernel = numpy.ones((7,7),numpy.uint8)

            tophat = cv2.morphologyEx(img, cv2.MORPH_TOPHAT, kernel)

            # Apply Gaussian blur to the image
            scene_blur = cv2.GaussianBlur(tophat,  # input image
                                          (3,3),    # kernel size
                                          0)          # automatically calculate the standard

            # deviation from the kernel size
            # Detect the edges
            scene_edges = cv2.Canny(scene_blur,  # input image
                                    100,         # low threshold for the hysteresis procedure
                                    200)         # high threshold for the hysteresis procedure

            closing = cv2.morphologyEx(scene_edges, cv2.MORPH_CLOSE, kernel)

            im2, contours, hierarchy = cv2.findContours(closing,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)


            image_center = numpy.array([80,100])
            img_sz = tuple(img.shape[0:2])
            distance_boxes = [1000]
            box_goals = [[0.0,0.0]]

            for i in contours:
                (x,y),radius = cv2.minEnclosingCircle(i)
                p_i = circle(int(x),int(y),int(radius))
                if  len(i) > 10  and int(y) > 30:
                    (x,y),radius = cv2.minEnclosingCircle(i)
                    center = (int(x),int(y))
                    radius = int(2*int(radius))

                    #create a mask
                    mask_circle = numpy.zeros(img_sz,dtype = numpy.uint8)
                    cv2.circle(mask_circle,center,radius,(255,255,255),-1,8,0)
                    cut = cv2.bitwise_and(img,img,mask = mask_circle)
                    #to hsv
                    hsv_cut = cv2.cvtColor(cut,cv2.COLOR_BGR2HSV)
                    blue_mask = cv2.inRange(hsv_cut,self.lower_blue, self.upper_blue)
                    blue_flag = sum(sum(blue_mask))/255
                    #print "number of blue pixels " , blue_flag
                    if blue_flag >1  and blue_flag < 18:
                        distance_boxes.append(image_center[1] - center[1])
                        box_goals.append(list(center))
                        found = 1

            if found:
                self.theo_found_box = True
                ind = distance_boxes.index(min(distance_boxes))
                self.box_far_away_coord_theo = image_center - numpy.array(box_goals[ind])
                img = cv2.circle(img,tuple(box_goals[ind]),10,(0,255,0),2)
            else:
                self.theo_found_box = False
                self.box_far_away_coord_theo = numpy.array([0,0])

        self.find_the_box_theo = False
        self.IO.imshow('img',img)



class circle:

    def __init__(self,x,y,r):

        self.x = x
        self.y = y
        self.r = r






