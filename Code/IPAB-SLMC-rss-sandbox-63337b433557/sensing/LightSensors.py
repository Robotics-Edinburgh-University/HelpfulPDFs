#!/usr/bin/env python

from Sensors import Sensors

#subclass of the Sensors class 
class LightSensors(Sensors):
    
    def __init__(self,io):
      
	Sensors.__init__(self, io )
          
       
    def measure_values_on_axis(self):
        
        self.update_analog_sensors_meas()    
	
	print "Analogs " , self.analogs_sensors