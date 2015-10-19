#!/usr/bin/env python

import unittest
import matplotlib.pyplot as plt
import numpy


import Map




class map_representation(unittest.TestCase):


    #def __init__(self,robot_map):
	#s = 0
	#self.robot_map = robot_map
    
    # tesp the map
    def test_map_plot(self):
        robot_map = Map.MapRoom()
        self.robot_map = robot_map
        points = numpy.asarray(self.robot_map.list_room_edges).T
        
        # plot outer frame
        plt.plot(points[0,:],points[1,:],marker='o', linewidth=5, linestyle='-', color='k' , label='Fence')

        # plot pilar
	if ( self.robot_map.obstacle != None ): 
	    obstacle_points = numpy.asarray(self.robot_map.obstacle.list_obstacle_edges).T
	    plt.plot(obstacle_points[0,:],obstacle_points[1,:],marker='+', linewidth=3, linestyle='-', color='b' , label='obstacle')
	
	# plot every room in red
	for room in self.robot_map.room_list:
	     
	    room_points = numpy.asarray(room.list_room_edges).T  
	    plt.plot(room_points[0,:],room_points[1,:],marker='*', linewidth=2, linestyle='-', color='r' , label=room.name)
	    
	    # plot obstacles in rooms 
	    if ( room.obstacle != None ): 
	       obstacle_points = numpy.asarray(room.obstacle.list_obstacle_edges).T
	       plt.plot(obstacle_points[0,:],obstacle_points[1,:],marker='+', linewidth=3, linestyle='-', color='b' , label='obstacle')
	    
	    
	    
	    
	plt.xlabel('X axis area')
	plt.ylabel('Y axis area')
	plt.title('Robot map')
	plt.xlim(-100,600)# 430)
	plt.ylim(-100,600)
	plt.legend()
        plt.show()
     
 
if __name__ == '__main__':
        
    unittest.main() 
    