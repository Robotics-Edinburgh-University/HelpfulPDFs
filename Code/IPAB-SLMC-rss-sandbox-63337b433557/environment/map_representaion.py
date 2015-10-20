#!/usr/bin/env python

import unittest
import matplotlib.pyplot as plt
import numpy
import copy

import Map




class map_representation:


    def __init__(self):
	self.robot_map = Map.MapRoom()
    
    def waypoint_computation(self):
        
        waypoint_list = []
        
        #waypoint_info = {'coord': ,
			 #'room': ,
			 #'name': ,
			   #'id': }
        
        # Room A 
        roomA = self.robot_map.room_list[0]
	waypoint_roomA_exit = (numpy.array(roomA.a) + numpy.array(roomA.e))/2.
	waypoint_roomA_middle = waypoint_roomA_exit/2.
	
	waypoint_info = {'coord': waypoint_roomA_exit,
			 'room': "A",
			 'name': "A",
			   'id': len(waypoint_list)}
	waypoint_list.append(waypoint_info)
	
	waypoint_info = {'coord': waypoint_roomA_middle,
			 'room': "A",
			 'name': "B",
			   'id': len(waypoint_list)}
	waypoint_list.append(waypoint_info)
	
	
	
	# Room B 
        roomB = self.robot_map.room_list[1]
	waypoint_roomB_exit = (numpy.array(roomB.a) + numpy.array(roomB.g))/2.
	waypoint_roomB_middle_1 = copy.deepcopy(waypoint_roomB_exit)
	waypoint_roomB_middle_1[0]  = waypoint_roomB_middle_1[0] + 91
	waypoint_roomB_middle_2 = copy.deepcopy(waypoint_roomB_exit)
	waypoint_roomB_middle_2[1]  = waypoint_roomB_middle_1[1] - 118
	
	waypoint_info = {'coord': waypoint_roomB_exit,
			 'room': "B",
			 'name': "C",
			   'id': len(waypoint_list)}
	waypoint_list.append(waypoint_info)
	
	waypoint_info = {'coord': waypoint_roomB_middle_1,
			 'room': "B",
			 'name': "D",
			   'id': len(waypoint_list)}
	waypoint_list.append(waypoint_info)
	
	waypoint_info = {'coord': waypoint_roomB_middle_2,
			 'room': "B",
			 'name': "E",
			   'id': len(waypoint_list)}
	waypoint_list.append(waypoint_info)
	
	
	# Room C 
        roomC = self.robot_map.room_list[2]
	waypoint_roomC_exit = (numpy.array(roomC.a) + numpy.array(roomC.d))/2.
	
	waypoint_info = {'coord': waypoint_roomC_exit,
			 'room': "C",
			 'name': "F",
			   'id': len(waypoint_list)}
	waypoint_list.append(waypoint_info)
	
	

	# Room D 
        roomD = self.robot_map.room_list[3]
	waypoint_roomD_exit = (numpy.array(roomD.a) + numpy.array(roomD.d))/2.
	
	waypoint_info = {'coord': waypoint_roomD_exit,
			 'room': "D",
			 'name': "G",
			   'id': len(waypoint_list)}
	waypoint_list.append(waypoint_info)
	
	
	# Room E 
        roomF = self.robot_map.room_list[4]
	waypoint_roomF_exit = (numpy.array(roomF.a) + numpy.array(roomF.g))/2.
	waypoint_roomF_middle_1 = copy.deepcopy(waypoint_roomF_exit)
	waypoint_roomF_middle_1[0]  = waypoint_roomF_middle_1[0] - 91
	waypoint_roomF_middle_2 = copy.deepcopy(waypoint_roomF_exit)
	waypoint_roomF_middle_2[1]  = waypoint_roomF_middle_1[1] + 118
	
	waypoint_info = {'coord': waypoint_roomF_exit,
			 'room': "E",
			 'name': "H",
			   'id': len(waypoint_list)}
	waypoint_list.append(waypoint_info)
	
	waypoint_info = {'coord': waypoint_roomF_middle_1,
			 'room': "E",
			 'name': "I",
			   'id': len(waypoint_list)}
	waypoint_list.append(waypoint_info)
	
	waypoint_info = {'coord': waypoint_roomF_middle_2,
			 'room': "E",
			 'name': "J",
			   'id': len(waypoint_list)}
	waypoint_list.append(waypoint_info)
	
	
	# Room F 
        roomF = self.robot_map.room_list[5]
	waypoint_roomF_exit = (numpy.array(roomF.a) + numpy.array(roomF.e))/2.
	waypoint_roomF_middle = (self.robot_map.c - waypoint_roomF_exit)/2. + waypoint_roomF_exit
	
	
	waypoint_info = {'coord': waypoint_roomF_exit,
			 'room': "F",
			 'name': "K",
			   'id': len(waypoint_list)}
	waypoint_list.append(waypoint_info)
	
	waypoint_info = {'coord': waypoint_roomF_middle,
			 'room': "F",
			 'name': "L",
			   'id': len(waypoint_list)}
	waypoint_list.append(waypoint_info)
	
	
	return waypoint_list
      
    # tesp the map
    def map_plot(self):
        #robot_map = Map.MapRoom()
        #self.robot_map = robot_map
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
	    plt.plot(room_points[0,:],room_points[1,:],marker='*', linewidth=2, linestyle='-', color=room.color , label=room.name)
	    
	    # plot obstacles in rooms 
	    if ( room.obstacle != None ): 
	       obstacle_points = numpy.asarray(room.obstacle.list_obstacle_edges).T
	       plt.plot(obstacle_points[0,:],obstacle_points[1,:],marker='+', linewidth=3, linestyle='-', color='b' , label='obstacle')
	    
	    # plot patches in rooms 
	    if ( room.patch != None ): 
	       patch_points = numpy.asarray(room.patch.list_patch_edges).T
	       plt.plot(patch_points[0,:],patch_points[1,:],marker='+', linewidth=5, linestyle='-', color='k' , label='patch')
	    
	waypoints = self.waypoint_computation()
	#waypoints_points = numpy.asarray(waypoints).T
	for waypoint in waypoints:
	    plt.plot((waypoint['coord'])[0],(waypoint['coord'])[1], marker='D', color='c' , label='waypoint')
	
	
	plt.xlabel('X axis area')
	plt.ylabel('Y axis area')
	plt.title('Robot map')
	plt.xlim(-100, 430)
	plt.ylim(-100,600)
	#plt.legend()
        plt.show()
     

class map_rep_test(unittest.TestCase):
    
    def test_map_rep_funcs(self):
        
        mp = map_representation()
        mp.map_plot()
     
           
      
if __name__ == '__main__':
        
    unittest.main() 
    