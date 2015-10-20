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
       	self.list_room_edges = [self.a, self.b , self.c, self.d, self.e]
       	
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
       	
        self.list_room_edges = [self.a, self.b , self.c, self.d]
       	
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
       	
        self.list_room_edges = [self.a, self.b , self.c, self.d]
       	
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
	self.list_room_edges = [self.a, self.b , self.c, self.d, self.e,self.f,self.g,self.h]

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
       	self.list_room_edges = [self.a, self.b , self.c, self.d, self.e]
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
	self.list_obstacle_edges = [self.a,self.b,self.c,self.d,self.a]


# Class for patch definition (start and end)
class Patch:
    	
    def __init__(self, a, b, c, d):
        self.a = a
        self.b = b
        self.c = c
        self.d = d
	self.list_patch_edges = [self.a,self.b,self.c,self.d,self.a]

