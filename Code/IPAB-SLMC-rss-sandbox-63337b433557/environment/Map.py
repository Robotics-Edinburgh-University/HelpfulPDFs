#!/usr/bin/env python

import numpy
import copy

import graph

class MapRoom:

    def __init__(self):
       	self.height = 536     # dimension along y axis of the map  
       	self.width = 323      # dimension along x axis of the map 
       	self.a = [0, 0]
       	self.b = [0, 536]
       	self.c = [323, 536]
       	self.d = [323, 0]
       	self.list_room_verteces = [self.a, self.b , self.c, self.d,self.a]
       	self.roomA = RoomA()		
       	self.roomB = RoomB()
       	self.roomC = RoomC()
       	self.roomD = RoomD()
        self.roomE = RoomE()
        self.roomF = RoomF()
        self.room_list = [self.roomA, self.roomB, self.roomC,self.roomD,self.roomE,self.roomF] 
	
	# pilar on the middle
	self.square_a = [180,283]
        self.square_b = [142,283]
        self.square_c = [142,250]
        self.square_d = [180,250]
       	self.obstacle = Obstacle(self.square_a,self.square_b,self.square_c,self.square_d)
       	
       	# represnt the map walls as lines 
       	self.wall_line_list = self.build_wall_as_line_segments()
       	
       	# compute the waypoints 
       	self.waypoints = self.waypoint_computation()
       	
       	# graph representation of the waypoints
       	self.waypoints_as_graph = graph.Graph()
       	
       	# add the nodes(waypoints) to the graph of the waypoints
	for i in self.waypoints:
            self.waypoints_as_graph.add_vertex(i['name'],i['coord'])
    
	# add the edges to the graph of the waypoints
	for i in self.waypoints:
	   for j in self.waypoints:
	       self.waypoints_as_graph.filter_edge_addition(i['name'], j['name'], self.wall_line_list)  
	  
	  
    # represent the map walls as lines 
    def build_wall_as_line_segments(self):
	wall_line_list = []
	for room in self.room_list:
	    temp_list = room.list_room_verteces
	    wall_line_list += self.pair_wise_list(temp_list)

	    if ( room.obstacle != None ): 
	       temp_list = room.obstacle.list_obstacle_verteces
	       wall_line_list += self.pair_wise_list(temp_list)
	    
	temp_list = self.obstacle.list_obstacle_verteces   
        wall_line_list += self.pair_wise_list(temp_list)
	
	
	return  wall_line_list 

    # make pairs out of a list --> denoting all connections
    # [1,2,3,4,5,6] --> [1,2],[2,3],[3,4],....
    def pair_wise_list(self,seq):
        return zip(seq, seq[1:])
    
    
    # buliding up all the import waypoints in the map
    def waypoint_computation(self):
        
        waypoint_list = []
        
        #waypoint_info = {'coord': ,
			 #'room': ,
			 #'name': ,
			   #'id': }
        
        # Room A 
        roomA = self.room_list[0]
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
        roomB = self.room_list[1]
	waypoint_roomB_exit = (numpy.array(roomB.a) + numpy.array(roomB.g))/2.
	waypoint_roomB_middle_1 = copy.deepcopy(waypoint_roomB_exit)
	waypoint_roomB_middle_1[0]  = waypoint_roomB_middle_1[0] + 88
	waypoint_roomB_middle_2 = copy.deepcopy(waypoint_roomB_exit)
	waypoint_roomB_middle_2[1]  = waypoint_roomB_middle_1[1] - 115
	
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
        roomC = self.room_list[2]
	waypoint_roomC_exit = (numpy.array(roomC.a) + numpy.array(roomC.d))/2.
	
	waypoint_info = {'coord': waypoint_roomC_exit,
			 'room': "C",
			 'name': "F",
			   'id': len(waypoint_list)}
	waypoint_list.append(waypoint_info)
	
	

	# Room D 
        roomD = self.room_list[3]
	waypoint_roomD_exit = (numpy.array(roomD.a) + numpy.array(roomD.d))/2.
	
	waypoint_info = {'coord': waypoint_roomD_exit,
			 'room': "D",
			 'name': "G",
			   'id': len(waypoint_list)}
	waypoint_list.append(waypoint_info)
	
	
	# Room E 
        roomE = self.room_list[4]
	waypoint_roomE_exit = (numpy.array(roomE.a) + numpy.array(roomE.g))/2.
	waypoint_roomE_middle_1 = copy.deepcopy(waypoint_roomE_exit)
	waypoint_roomE_middle_1[0]  = waypoint_roomE_middle_1[0] - 88
	waypoint_roomE_middle_2 = copy.deepcopy(waypoint_roomE_exit)
	waypoint_roomE_middle_2[1]  = waypoint_roomE_middle_1[1] + 115
	
	waypoint_info = {'coord': waypoint_roomE_exit,
			 'room': "E",
			 'name': "H",
			   'id': len(waypoint_list)}
	waypoint_list.append(waypoint_info)
	
	waypoint_info = {'coord': waypoint_roomE_middle_1,
			 'room': "E",
			 'name': "I",
			   'id': len(waypoint_list)}
	waypoint_list.append(waypoint_info)
	
	waypoint_info = {'coord': waypoint_roomE_middle_2,
			 'room': "E",
			 'name': "J",
			   'id': len(waypoint_list)}
	waypoint_list.append(waypoint_info)
	
	
	# Room F 
        roomF = self.room_list[5]
	waypoint_roomF_exit = (numpy.array(roomF.a) + numpy.array(roomF.e))/2.
	waypoint_roomF_middle = (self.c - waypoint_roomF_exit)/2. + waypoint_roomF_exit
	
	
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
	
	
	# Map waypoints 
	p_interest = filter( lambda waypoint: waypoint['name']=='C' or waypoint['name']=='F', waypoint_list )
	#pA_interest = filter( lambda waypoint: waypoint['name']=='A', waypoint_list )
	#pC_interest = filter( lambda waypoint: waypoint['name']=='C', waypoint_list )
	#print "ddd " , (pC_interest[0])['coord']
	#waypoint_middle_south = numpy.array([((pA_interest[0])['coord'])[0] , ((pC_interest[0])['coord'])[1]])
	waypoint_middle_south = ((p_interest[0])['coord'] + (p_interest[1])['coord'])/2
	waypoint_info = {'coord': waypoint_middle_south,
			 'room': "S",
			 'name': "M",
			   'id': len(waypoint_list)}
	waypoint_list.append(waypoint_info)  
	
	p_interest = filter( lambda waypoint: waypoint['name']=='H' or waypoint['name']=='G', waypoint_list )
	waypoint_middle_south = ((p_interest[0])['coord'] + (p_interest[1])['coord'])/2
	waypoint_info = {'coord': waypoint_middle_south,
			 'room': "N",
			 'name': "N",
			   'id': len(waypoint_list)}
	waypoint_list.append(waypoint_info)  
	
	
	waypoint_column_right = (self.obstacle.d + numpy.array(roomB.g))/2
	waypoint_info = {'coord': waypoint_column_right,
			 'room': "C",
			 'name': "Y",
			   'id': len(waypoint_list)}
	waypoint_list.append(waypoint_info)  
	
	waypoint_column_right = (self.obstacle.b + numpy.array(roomE.g))/2
	waypoint_info = {'coord': waypoint_column_right,
			 'room': "C",
			 'name': "Z",
			   'id': len(waypoint_list)}
	waypoint_list.append(waypoint_info)
	
	#print waypoint_list
	return waypoint_list
    
   
class RoomA:

    def __init__(self):
       	self.name = "roomA"
       	self.color = 'r' 
      	self.start_room = None  
        self.goal_room = None
       	self.width = 145
      	self.height = 133
       	self.a = [75,133]
       	self.b = [0,133]
       	self.c = [0,0]
       	self.d = [145,0]
       	self.e = [145,94]
       	self.list_room_verteces = [self.a, self.b , self.c, self.d, self.e]
       	
       	# base patch
       	self.patch_a = [40,60]
        self.patch_b = [20,60]
        self.patch_c = [20,20]
        self.patch_d = [40,20]
       	self.patch = Patch(self.patch_a,self.patch_b,self.patch_c,self.patch_d)
       
       	self.numberOfObjects = 1
       	self.object1 = MyObject("Lego Tower, Glass Lego on Top","BLUE",4,[15,120],"GLASS LEGO") 
	self.obstacle = None

class RoomB:

    def __init__(self):
        self.name = "roomB"
        self.color = 'g' 
       	self.start_room = None
        self.goal_room = None
        self.patch = None
        self.width = 178
       	self.height = 250
       	self.a = [176,94]
       	self.b = [145,94]
       	self.c = [145,0]
       	self.d = [323,0]  
       	self.e = [323,250] 
       	self.f = [229,250]
       	self.g = [229,199]
       	self.h = [195,199]
	self.list_room_verteces = [self.a, self.b , self.c, self.d, self.e,self.f,self.g,self.h]

       	# obstacle obecjt in the room 
        self.square_a = [232,95]
        self.square_b = [232,118]
        self.square_c = [255,118]
        self.square_d = [255,95]
       	self.obstacle = Obstacle(self.square_a,self.square_b,self.square_c,self.square_d)
       	
       	
        self.numberOfObjects = 3
       	self.object1 = MyObject("Lego Tower","YELLOW",4,[335,15],None)
       	self.object2 = MyObject("Lego Tower","GREEN",4,[310,15],None)
       	self.object3 = MyObject("Lego Tower","ORAGNE",5,[310,230],None)


class RoomC:
    def __init__(self):
        self.name = "roomC"
        self.color = 'm' 
        self.start_room = None
        self.goal_room = None
        self.patch = None
       	self.width = 94
       	self.height = 162
       	self.start_room = None
        self.goal_room = None
        self.a = [94,293]
       	self.b = [0,293]
        self.c = [0,133]
       	self.d =  [75,133]
       	
        self.list_room_verteces = [self.a, self.b , self.c, self.d]
       	
       	self.numberOfObjects = 3
       	self.object1 = MyObject("Lego Tower","GREEN",3,[20,145],"COW")
       	self.object2 = MyObject("Lego Tower","WHITE",4,[15,200],"CHICKEN")
       	self.object3 = MyObject("Lego Tower","GREEN",4,[85,270],"PING")
        self.obstacle = None



class RoomD:
    def __init__(self):
        self.name = "roomD"
        self.color = 'm' 
       	self.start_room = None
        self.goal_room = None
        self.patch = None
        self.width = 94
       	self.height = 150
       	self.start_room = None
        self.goal_room = None
        self.a = [229,247]
       	self.b = [323,247]
        self.c = [323,394]
       	self.d =  [248,394]
       	
        self.list_room_verteces = [self.a, self.b , self.c, self.d]
       	
       	self.numberOfObjects = 3
       	self.object1 = MyObject("Lego Tower","YELLOW",3,[20,145],"COW")
       	self.object2 = MyObject("Lego Tower","BLACK",4,[15,200],"CHICKEN")
       	self.object3 = MyObject("Lego Tower","BLUE",4,[85,270],"PING")
        self.obstacle = None


class RoomE:

    def __init__(self):
        self.name = "roomE"
        self.color = 'g' 
       	self.start_room = None
        self.goal_room = None
        self.patch = None
        self.width = 173
       	self.height = 240
       	self.a = [141,443]
       	self.b = [173,443]
       	self.c = [173,536]
       	self.d = [0,536]  
       	self.e = [0,296] 
       	self.f = [93,296]
       	self.g = [93,343]
       	self.h = [127,343]
	self.list_room_verteces = [self.a, self.b , self.c, self.d, self.e,self.f,self.g,self.h]

       	# obstacle obecjt in the room 
        self.square_a = [68,441]
        self.square_b = [68,418]
        self.square_c = [91,418]
        self.square_d = [91,441]
       	self.obstacle = Obstacle(self.square_a,self.square_b,self.square_c,self.square_d)
       	
       	
        self.numberOfObjects = 3
       	self.object1 = MyObject("Lego Tower","YELLOW",4,[335,15],None)
       	self.object2 = MyObject("Lego Tower","GREEN",4,[310,15],None)
       	self.object3 = MyObject("Lego Tower","ORAGNE",5,[310,230],None)


	
class RoomF:

    def __init__(self):
       	self.name = "roomF"
       	self.color = 'r' 
      	self.start_room = None  
        self.goal_room = None
        self.patch = None
       	self.width = 149
      	self.height = 139
       	self.a = [248,397]
       	self.b = [323,397]
       	self.c = [323,536]
       	self.d = [174,536]
       	self.e = [174,443]
       	self.list_room_verteces = [self.a, self.b , self.c, self.d, self.e]
       	self.numberOfObjects = 1
       	self.object1 = MyObject("Lego Tower, Glass Lego on Top","BLUE",4,[15,120],"GLASS LEGO") 
	self.obstacle = None


# Class for target object definition
class MyObject:
	
    def __init__(self,name,color,height,(x,y),animal):
        self.name = name
       	self.color = color
       	self.animal = animal
       	self.height = height
       	self.coordinates = (x,y)	#estimated position


# Class for obstacle object definition
class Obstacle:
    	
    def __init__(self, a, b, c, d):
        self.a = a
        self.b = b
        self.c = c
        self.d = d
	self.list_obstacle_verteces = [self.a,self.b,self.c,self.d,self.a]


# Class for patch definition (start and end)
class Patch:
    	
    def __init__(self, a, b, c, d):
        self.a = a
        self.b = b
        self.c = c
        self.d = d
	self.list_patch_verteces = [self.a,self.b,self.c,self.d,self.a]

