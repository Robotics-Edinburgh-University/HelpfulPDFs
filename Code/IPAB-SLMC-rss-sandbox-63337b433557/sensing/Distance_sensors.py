#!/usr/bin/env python

from Sensors import Sensors

# calss for all the distance sensors
class Distance_sensors(object):
    
    def __init__(self, io ):
      
	Sensors.__init__(self, io )
	
	
	#Collision to the left IR                                         
	self.LeftIRcollision = (0,0)
	
	#Collision to the right IR
	self.RightIRcollision = (0,0)
	
	#Collision to the Sonar
	self.SonarCollison = (0,0)
	
	# Thresholds of its sensor
	self.left_IR_limit = 480
	self.right_IR_limit = 480
	self.sonar_limit = 20
	
    
    def from_IR_readings_2_distance(self):
        print "DEVELOPMENT OF THIS FUNCION IS MISSING!!!!!! " 
        print "develope the function which transform IR measurements to distance!!!"  
        distance = 0 
        return distance 
      
      
    # set the limits of the sensors and the directions    
    def define_limits_of_sensors_2_directions(self):  
        
        self.from_IR_readings_2_distance()
        
        self.LeftIRcollision = (0,-1) if self.analogs_sensors[0] >= self.left_IR_limit else (0,0)
        self.RightIRcollision = (0,1) if self.analogs_sensors[7] >= self.right_IR_limit else (0,0)
        self.SonarCollision = (-1,0) if self.analogs_sensors[6] <= self.sonar_limit else (0,0)
	                        
    # provide the overall direction out of the distance sensor	
    def return_direction_IR_Sonar(self):
        
        self.update_analog_sensors_meas()
        
        self.define_limits_of_sensors_2_directions()
        
	#Adding coordinates 
	#(1,2) + (10,10) + (20,20) = (31,32)
	# vectors orientation, magnitude is always 1
        return tuple(map(sum,zip(self.LeftIRcollision,self.RightIRcollision,self.SonarCollision)))
    
    
    
   