
#!/usr/bin/env python


class FindRoomByColor:
	
    def __init__(self):

        self.roomA = [Colors.blue]
        self.roomB = [Colors.yellow, Colors.green, Colors.orange]
        self.roomC = [Colors.green, Colors.white, Colors.green]
        self.roomD = [Colors.yellow, Colors.black, Colors.blue]
        self.roomE = [Colors.red, Colors.blue, Colors.red, Colors.black]
        self.roomF = [Colors.green]

    def returnRoom(self, colorList):

        colors_found = []
        counter = 0
        for color in colorList:
            if color != 0:
                  colors_found.append(counter)
            counter += 1
        rooms = []
        if len(colors_found) > 1:
            if Colors.red in colors_found:
                return ["E"]

            if Colors.orange in colors_found:
                return ["B"]
            if (Colors.yellow in colors_found) and (Colors.green in colors_found):
                return ["B"]

            if (Colors.yellow in colors_found) and (Colors.black in colors_found):
                return ["D"]
            if (Colors.yellow in colors_found) and (Colors.blue in colors_found):
                return ["D"]

          #  if Colors.white in colors_found:
          #      return ["C"]

        else:
            if Colors.red in colors_found:
                return ["E"]
         #   if Colors.white in colors_found:
         #       return ["C"]
            if Colors.orange in colors_found:
                return ["B"]
            if Colors.yellow in colors_found:
                rooms.append("B")
                rooms.append("D")
                return rooms
            if Colors.blue in colors_found:
                rooms.append("A")
                rooms.append("E")
                rooms.append("D")
                return rooms
            if Colors.green in colors_found:
                rooms.append("B")
                rooms.append("C")
                rooms.append("F")

        return rooms

    def update_rooms(self,estimatedRooms,latestRoomsFromColor):

        if len(estimatedRooms) == 0:
            return latestRoomsFromColor
        else:
            if len(latestRoomsFromColor) != 0:
                estimatedRooms = list(set(estimatedRooms).intersection(latestRoomsFromColor))
        return estimatedRooms


class Colors:
    red = 0
    green = 1
    blue = 2
    yellow = 3
    orange = 4
    white = 5
    black = 6

    def CodeToColor(self,number):
        if number == 0:
            return "Red"
        elif number == 1:
            return "Green"
        elif number == 2:
            return "Blue"
        elif number == 3:
            return "Yellow"
        elif number == 4:
            return "Orange"
########################################

#findRoom = FindRoomByColor()
#estimatedRooms = []

#latestRoomsFromColor = findRoom.returnRoom([0,0,0,1,0,0,0])
#print latestRoomsFromColor
#estimatedRooms = findRoom.update_rooms(estimatedRooms,latestRoomsFromColor)
#latestRoomsFromColor = findRoom.returnRoom([0,1,0,0,0,0,1])
#estimatedRooms = findRoom.update_rooms(estimatedRooms,latestRoomsFromColor)
#print estimatedRooms















#list(set(list1).intersection(list2))
