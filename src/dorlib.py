#!/usr/bin/env python3
#from numpy import linalg, array

from math import sin, cos, radians,ceil           
def dCoordsInRad(rad,resolution = 3):  
    step = radians(90)/resolution                     
    maxes_list = []
    last_x = 0
    list = []
    for num in range(round(radians(90)/step) + 1):   
        x_max = ceil(cos(num*step)*rad)
        y_max = ceil(sin(num*step)*rad)
        maxes_list.append((x_max,y_max))
    last_x = 0
    for x_max, y_max in maxes_list:
        for x in range(last_x,x_max):
            for y in range(y_max):
                list.append((x,y))
                list.append((x,-y))
                list.append((-x,y))
                list.append((-x,-y))
        last_x = x_max
    return list



def dCoordsOnCircle(rad,resolution = 3):
    step = radians(180)/resolution
    maxes= []
    list = []
    for num in range(round(radians(180)/step) + 1):
        maxes.append((rad*sin(num*step),rad*cos(num*step)))
    for x,y in maxes:
        list.append((x,y))
        list.append((x,-y))
    return list
