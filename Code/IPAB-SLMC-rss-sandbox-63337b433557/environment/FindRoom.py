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

        rooms = []
        for color in colorList:
            if color in self.roomA:
                rooms.append("A")
            if color in self.roomB:
                rooms.append("B")
            if color in self.roomC:
                rooms.append("C")
            if color in self.roomD:
                rooms.append("D")
            if color in self.roomE:
                rooms.append("E")
            if color in self.roomF:
                rooms.append("F")
        return rooms





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

#list(set(list1).intersection(list2))