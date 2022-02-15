#!/usr/bin/env python3
from numpy import linalg, array
from math import sin, cos, radians,ceil           
def deltaCoordsInRad(rad,step):  
    step = step - radians(90)%step                      
    maxes_list = []
    last_x = 0
    for num in range(radians(90)//step + 1):   
        x_max = ceil(cos(num*step)*rad)
        y_max = ceil(sin(num*step)*rad)
        for x in range(last_x,x_max):
            maxes_list.append((x, y_max))
        last_x = x_max
    for x_max, y_max in maxes_list:
        for x in range(-x_max,x_max):
            for y in range(-y_max,y_max):
                yield (x,y)
def deltaCoordsOnRad(rad,step):
    step = step - radians(360)%step
    maxes= []
    list = []
    for num in range(radians(180)//step + 1):
        maxes.append((rad*sin(num*step),rad*cos(num*step)))
    for x,y in maxes:
        list.append((x,y))
        list.append((x,-y))
    return list
