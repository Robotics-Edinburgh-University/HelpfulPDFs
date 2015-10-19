#!/usr/bin/env python

class MapRoom:

    def __init__(self):
       	self.height = 536     # dimension along y axis of the map  
       	self.width = 323      # dimension along x axis of the map 
       	self.a = [0, 0]
       	self.b = [0, 536]
       	self.c = [323, 536]
       	self.d = [323, 0]
       	self.list_room_edges = [self.a, self.b , self.c, self.d,self.a]
       	self.roomA = RoomA()		
       	self.roomB = RoomB()
       	self.roomC = RoomC()
       	#self.roomD = RoomD()
        #self.roomF = RoomF()
        #self.roomE = RoomE()
        self.room_list = [self.roomA, self.roomB, self.roomC] 
	
	# pilar on the middle
	self.square_a = [166,283]
        self.square_b = [128,283]
        self.square_c = [128,252]
        self.square_d = [166,252]
       	self.obstacle = Obstacle(self.square_a,self.square_b,self.square_c,self.square_d)
       	
	
	
class RoomA:

    def __init__(self):
       	self.name = "roomA"
      	self.start_room = None  
        self.goal_room = None
       	self.width = 145
      	self.height = 133
       	self.a = [75,133]
       	self.b = [0,133]
       	self.c = [0,0]
       	self.d = [145,0]
       	self.e = [145,94]
       	self.list_room_edges = [self.a, self.b , self.c, self.d, self.e]
       	self.numberOfObjects = 1
       	self.object = MyObject("Lego Tower, Glass Lego on Top","BLUE",4,[15,120],"GLASS LEGO") 
	self.obstacle = None

class RoomB:

    def __init__(self):
        self.name = "roomB"
       	self.start_room = None
        self.goal_room = None
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
	self.list_room_edges = [self.a, self.b , self.c, self.d, self.e,self.f,self.g,self.h]

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
       	self.width = 94
       	self.height = 150
       	self.start_room = None
        self.goal_room = None
        self.a = [94,283]
       	self.b = [0,283]
        self.c = [0,133]
       	self.d =  [75,133]
       	
        self.list_room_edges = [self.a, self.b , self.c, self.d]
       	
       	self.numberOfObjects = 3
       	self.object1 = MyObject("Lego Tower","GREEN",3,[20,145],"COW")
       	self.object2 = MyObject("Lego Tower","WHITE",4,[15,200],"CHICKEN")
       	self.object3 = MyObject("Lego Tower","GREEN",4,[85,270],"PING")
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
	self.list_obstacle_edges = [self.a,self.b,self.c,self.d,self.a]



