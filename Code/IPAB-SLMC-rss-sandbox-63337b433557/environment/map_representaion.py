#!/usr/bin/env python


import numpy

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
    
    # returns the interpolated path
    def path_coordinates(self,path):
        
        coord_path = []
        for node in path:
	    coord = (self.robot_map.waypoints_as_graph.get_vertex(node)).coord
	    coord_path.append([int(coord[0]),int(coord[1])])
	
	return coord_path
	
    # returns the interpolated path
    def test_waypoint_interpolation(self,path):
        
        coord_path = []
        for node in path:
	    coord = (self.robot_map.waypoints_as_graph.get_vertex(node)).coord
	    coord_path.append(numpy.array([int(coord[0]),int(coord[1])]))
	
	relative_coord_path = [coord_path[0]]
	for i in xrange(len(coord_path)-1): 
	    relative_coord_path.append(coord_path[i+1] - coord_path[i]) 
	
	#print coord_path
	#print relative_coord_path
	for subpath in relative_coord_path:
	    print subpath
	    xsteps = abs(subpath[0])/5
	    x = numpy.sign(subpath[0])* 5.0
	    xpath = [[x,0.0,0.0] for _ in xrange(xsteps)]
	    if subpath[1] > 0.0 :
	        ypath_init = [0.0, 0.001 , 0.0]
	    if subpath[1] < 0.0 :
	        ypath_init = [0.0, -0.001 , 0.0]
	        
	    ysteps = abs(subpath[1])/5
	    y = numpy.sign(subpath[1])* 5.0
	    ypath  = [ypath_init] + [[y,0.0,0.0] for _ in xrange(ysteps)] + [[ypath_init[0],(ypath_init[1]*(-1)),ypath_init[2]]]	 
	    
            print xpath
            print "---------------------------------"
            print ypath
            raw_input("ddddd")
            
   
     

    
