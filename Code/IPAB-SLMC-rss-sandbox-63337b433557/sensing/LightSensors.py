#!/usr/bin/env python

from Sensors import Sensors
import math

#subclass of the Sensors class 
class LightSensors(Sensors):
    
    def __init__(self,io):						#Initialisation
      
	Sensors.__init__(self, io )
        self.X_difference = 0.0						#Reading difference of light sensors in X-axis
	self.Y_difference = 0.0						#Reading difference of light sensors in X-axis
	self.current_reading_of_light_sensors = [0.0, 0.0, 0.0, 0.0]	#Variable for collecting current readings of Light sensors
        self.X_diff_normalised = 0.0
	self.Y_diff_normalised = 0.0


    def measure_values_on_axis(self):					#measurement method
        
        self.update_analog_sensors_meas()				#update readings
	self.current_light_sensors_readings=self.analogs_sensors[0:4] #store updated readings into the Current Reading variable for further usage
	print "Analogs " , self.current_light_sensors_readings

    def calculate_light_sensor_difference(self):				#Method for calculating sensor differences
	
        self.measure_values_on_axis()					#obtain the light sensor readings
	self.X_difference=self.current_light_sensors_readings[0]-self.current_light_sensors_readings[1]	#Calculating X axis
	#self.Y_difference=self.current_light_sensors_readings[2]-self.current_light_sensors_readings[3]	#Calculating Y axis
	
	#self.X_diff_normalised = self.X_difference/math.sqrt(math.pow(self.X_difference,2)+math.pow(self.Y_difference,2))
	#self.X_diff_normalised = self.X_difference/math.sqrt(math.pow(self.X_difference,2)) #+math.pow(self.Y_difference,2))
	
	#self.Y_diff_normalised = self.Y_difference/math.sqrt(math.pow(self.X_difference,2)+math.pow(self.Y_difference,2))      
	
	#print "Normalised X difference ", self.X_diff_normalised
 	#print "Normalised Y difference ", self.Y_diff_normalised
	print "X axis difference ", self.X_difference
	#print "Y axis difference ", self.Y_difference
	
	
