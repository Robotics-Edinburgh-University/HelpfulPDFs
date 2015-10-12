#!/usr/bin/env python

# Mother class to be used only for inheritange reasons
class Sensors(object):
    
    def __init__(self, io ):
      
	# initialisation of the IO
	self.IO = io                                         
	
	# initialisation of the digital sensors  status vector 
        self.digital_sensors = [ False for i in xrange(8)]  
	# initialisation of the analog sensors status vector 
	self.analogs_sensors = [ 0.0 for i in xrange(8)]      
	 
    # Function for retrieving the digital sensor values
    def update_digital_sensors_meas(self):
	self.digital_sensors = self.IO.getInputs()

    # Function for retrieving the analog sensor values
    def update_analog_sensors_meas(self):
	self.analogs_sensors = self.IO.getSensors() 
    
    def overall_sensors_direction:

        sensors_IR = self.distance_sensors.return_direction_IR_Sonar_Sensors()
	sensors_Button = self.button_sensors.return_direction_Button_Sensors()            
        tuple(map(sum,zip(sensors_IR,sensors_Button)))
	
	
