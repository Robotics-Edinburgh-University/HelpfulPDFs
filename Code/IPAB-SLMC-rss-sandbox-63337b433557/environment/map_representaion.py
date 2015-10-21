#!/usr/bin/env python

import unittest
import matplotlib.pyplot as plt
import numpy
import copy

import Map




class map_representation:


    def __init__(self):
	self.robot_map = Map.MapRoom()
    
    def walls(self):
        self.robot_map.build_wall_as_line_segments()
    
    
    def retrieve_shortest_path(self,start_node ,goal_end):
        
        self.robot_map.waypoints_as_graph.dijkstra(self.robot_map.waypoints_as_graph.get_vertex(start_node), self.robot_map.waypoints_as_graph.get_vertex(goal_end)) 

	target = self.robot_map.waypoints_as_graph.get_vertex(goal_end)
	path = [target.get_id()]
	self.robot_map.waypoints_as_graph.shortest(target, path)
	#print 'The shortest path : %s' %(path[::-1])
        return path[::-1]
      
    # tesp the map
    def map_plot(self,plot_path = False, path = None):
        #robot_map = Map.MapRoom()
        #self.robot_map = robot_map
        points = numpy.asarray(self.robot_map.list_room_verteces).T
        
        # plot outer frame
        plt.plot(points[0,:],points[1,:],marker='o', linewidth=5, linestyle='-', color='k' , label='Fence')

        # plot pilar
	if ( self.robot_map.obstacle != None ): 
	    obstacle_points = numpy.asarray(self.robot_map.obstacle.list_obstacle_verteces).T
	    plt.plot(obstacle_points[0,:],obstacle_points[1,:],marker='+', linewidth=3, linestyle='-', color='b' , label='obstacle')
	
	# plot every room in red
	for room in self.robot_map.room_list:
	     
	    room_points = numpy.asarray(room.list_room_verteces).T  
	    plt.plot(room_points[0,:],room_points[1,:],marker='*', linewidth=2, linestyle='-', color=room.color , label=room.name)
	    
	    # plot obstacles in rooms 
	    if ( room.obstacle != None ): 
	       obstacle_points = numpy.asarray(room.obstacle.list_obstacle_verteces).T
	       plt.plot(obstacle_points[0,:],obstacle_points[1,:],marker='+', linewidth=3, linestyle='-', color='b' , label='obstacle')
	    
	    # plot patches in rooms 
	    if ( room.patch != None ): 
	       patch_points = numpy.asarray(room.patch.list_patch_verteces).T
	       plt.plot(patch_points[0,:],patch_points[1,:],marker='+', linewidth=5, linestyle='-', color='k' , label='patch')
	    
	#waypoints = self.waypoint_computation()
	#waypoints_points = numpy.asarray(waypoints).T
	for waypoint in self.robot_map.waypoints:
	    plt.plot((waypoint['coord'])[0],(waypoint['coord'])[1], marker='D', color='c' , label='waypoint')
	    plt.annotate(waypoint['name'],((waypoint['coord'])[0],(waypoint['coord'])[1]),((waypoint['coord'])[0] + 0.1,(waypoint['coord'])[1]- 0.1))
	
	
	# plot the graph of the waypoints
	plot_graph = False
	if plot_graph:
	    for j in self.robot_map.waypoints_as_graph.get_vertices():
		node_coord = (self.robot_map.waypoints_as_graph.get_vertex(j)).coord
		adv =  self.robot_map.waypoints_as_graph.vert_dict[j].adjacent
		for i in adv:
		    #print "in " , i.id 
		    neighboor_coord = i.coord
		    px = [node_coord[0],neighboor_coord[0]]
		    py = [node_coord[1],neighboor_coord[1]]
		    plt.plot(px,py)
	
	# plot connections of node
	plot_connections_node = False
	if plot_connections_node:	
	    j = 'A'
	    node_coord = (self.robot_map.waypoints_as_graph.get_vertex(j)).coord
	    adv =  self.robot_map.waypoints_as_graph.vert_dict[j].adjacent
	    for i in adv:
		print "in " , i.id 
		neighboor_coord = i.coord
		px = [node_coord[0],neighboor_coord[0]]
		py = [node_coord[1],neighboor_coord[1]]
		plt.plot(px,py, color='r')
	
	# plot solution path
	if plot_path:
	    linex = []
	    liney = []
	    for node in path:
	        node_coord = (self.robot_map.waypoints_as_graph.get_vertex(node)).coord
		linex.append(node_coord[0])  
		liney.append(node_coord[1])  
	    plt.plot(linex,liney, color='r')	
	    
  
	plt.xlabel('X axis area')
	plt.ylabel('Y axis area')
	plt.title('Robot map')
	plt.xlim(-100, 600) #430)
	plt.ylim(-100,600)
	#plt.legend()
        plt.show()
     

class map_rep_test(unittest.TestCase):
    
    # plot the map with each waypoints
    def t_test_map_rep_funcs(self):
        
        mp = map_representation()
        mp.map_plot()
    
    # test the building of the walls as lines
    def t_test_map_build(self):
        
        mp = map_representation()
        mp.walls()
    
    
    # test the computation of the shortest path
    def test_shortest_path_solution(self):
        
        mp = map_representation()
        plot_path = True
        solution_path = mp.retrieve_shortest_path('E','I')
        mp.map_plot(plot_path,solution_path)
    
      
if __name__ == '__main__':
        
    unittest.main() 
    