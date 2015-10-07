#!/usr/bin/env python
__TODDLER_VERSION__="1.0.0"

import time
import numpy
import cv2
import datetime
import os
import sys
import signal


# Get path of to the toddler file... to use always relative paths
CURRENT_PATH = os.getcwd() 

# import sensing modules
sys.path.insert(0, CURRENT_PATH + '/sensing')
from ButtonSensor import ButtonSensor
from LightSensors import LightSensors

# Hardware test code
class Toddler:
    def __init__(self,IO):
        print 'I am a toddler playing in a sandbox'
        self.IO=IO
        self.inp=[0, 0, 0, 0, 0, 0, 0, 0]
        
        self.buttonSensor = ButtonSensor(self.IO)
        self.lightSensor = LightSensors(self.IO)
       
       

    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Control(self, OK):
	
	# Move forward and backward in collision
        # self.buttonSensor.move_back_due_collision()
	while (1):	
	    self.lightSensor.measure_values_on_axis()
	
	
    # This is a callback that will be called repeatedly.
    # It has its dedicated thread so you can keep block it.
    def Vision(self, OK):
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
            

    def stop_motors_on_interupt(self):
        print('You pressed Ctrl+C!')
        time.sleep(1)
        self.IO.setMotors(0,0)
        time.sleep(1)
        self.IO.setMotors(0,0)


