#!/usr/bin/env python

from Sensors import Sensors
from random import randint
# calss for all the distance sensors
class Distance_sensors(Sensors):
    
    def __init__(self, io ):
      
	Sensors.__init__(self, io )
	
	#Collision to the left IR                                         
	self.LeftIRcollision = (0,0)
	
	#Collision to the right IR
	self.RightIRcollision = (0,0)
	
	#Collision to the Sonar
	self.SonarCollison = (0,0)
	
	# Thresholds of its sensor
	self.left_IR_limit = 414 #15 #480
	self.right_IR_limit = 414
	self.sonar_limit = 22 # 22
	
    # Transform IR measurements to distance
    def from_IR_readings_2_distance(self,SensorValue):  
        if  SensorValue == 20.0:
	    distance = 0.0
	else:    
            distance = 4800.0/(SensorValue - 20.0)
        return distance 
      
      
    # set the limits of the sensors and the directions    
    def update_direction(self):
      
        #left_distance = 100
        #right_distance = 100
        
        #left_distance = self.from_IR_readings_2_distance(self.analogs_sensors[0])
        #right_distance = self.from_IR_readings_2_distance(self.analogs_sensors[7])
        
        #self.LeftIRcollision = (0,-1) if left_distance <= self.left_IR_limit else (0,0)
        #self.RightIRcollision = (0,1) if right_distance <= self.right_IR_limit else (0,0)
        #self.SonarCollision = (-1,0) if self.analogs_sensors[6] <= self.sonar_limit else (0,0)
	
	self.LeftIRcollision = (0,-1) if self.analogs_sensors[0] >= self.left_IR_limit else (0,0)
        self.RightIRcollision = (0,1) if self.analogs_sensors[7] >= self.right_IR_limit else (0,0)
        random = randint(0,10)
        if random >= 5:
	    self.SonarCollision = (-1,1) if self.analogs_sensors[6] <= self.sonar_limit else (0,0)
	else:
	    self.SonarCollision = (-1,1) if self.analogs_sensors[6] <= self.sonar_limit else (0,0)
	    ################################# (-1,-1)!! changed

    # provide the overall direction out of the distance sensor	
    def return_direction_IR_Sonar_Sensors(self):
        
        self.update_analog_sensors_meas()
        
        self.update_direction()
        
	#Adding coordinates 
	#(1,2) + (10,10) + (20,20) = (31,32)
        return tuple(map(sum,zip(self.LeftIRcollision,self.RightIRcollision,self.SonarCollision)))
    
    
    
   
