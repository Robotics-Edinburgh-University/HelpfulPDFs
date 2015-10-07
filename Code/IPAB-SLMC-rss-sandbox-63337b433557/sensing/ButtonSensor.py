#!/usr/bin/env python

class ButtonSensor:
    
    def __init__(self):
        self.button_1_pressed = False
        self.button_2_pressed = False
        self.button_3_pressed = False
        self.button_4_pressed = False

#btn = 1,2,3,4 for each button ID
    def setButtonStatus(self,status):
        self.button_1_pressed = status[0]
        self.button_2_pressed = status[1]
        self.button_3_pressed = status[2]
        self.button_4_pressed = status[3]

#check for collision

    def collisionCheck(self):
        L=[]
        if self.button_1_pressed == True:
            L.append(1)
        if self.button_2_pressed == True:
            L.append(2)
        if self.button_3_pressed == True:
            L.append(3)
        if self.button_4_pressed == True:
            L.append(4)
        return L

