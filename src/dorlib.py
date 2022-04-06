#!/usr/bin/env python3
#from numpy import linalg, array
#from operator import ne
import numpy as np
from math import sin, cos, radians,ceil           
def dCoordsInRad(rad: int,resolution:int = 3):  
    step = radians(90)/resolution                     
    maxes_list = []
    list = []
    #print(f"Fetching all coords in radius {rad}, with res {resolution}")
    for num in range(round(radians(90)/step) + 1):   
        x_max = ceil(sin(num*step)*rad)   
        y_max = ceil(cos(num*step)*rad)
        #print(f"x_max = {x_max}, y_max = {y_max}")
        maxes_list.append((x_max,y_max))
    #print(f"Done!")
    last_x = -1
    for x_max, y_max in maxes_list:
        #print(f"cycling x_max = {x_max}, y_max = {y_max}")
        for x in range(last_x+1,x_max+1):
            for y in range(y_max+1):
                #print(f"x = {x},y = {y}")
                list.append((x,y))
                if y > 0:  
                    list.append((x,-y))
                if x > 0:
                    list.append((-x,y))
                    if y > 0: 
                        list.append((-x,-y))
        last_x = x_max
    #print (list)
    return list
def getRotor(turn):
    return cos(turn) + 1j*sin(turn)
def applyRotor(vect,rotor):
    comp = rotor * (vect[0] + 1j * vect[1])
    return (comp.real , comp.imag)
def turnVect(vect,turn: float, dims = 2):
    "Turn vector using complex rotor by (turn) rads"
    rotor = cos(turn) + 1j*sin(turn)
    if dims == 2:
        comp = rotor * (vect[0] + 1j * vect[1])
        new_vect = (comp.real , comp.imag)
    elif dims == 3:
        comp = rotor * (vect[0] + 1j * vect[1])
        new_vect = (comp.real , comp.imag, vect[2])
    else: 
        return "error"
    return new_vect
 
def dCoordsOnCircle(rad: int,resolution:int = 16): #in cells (res = number of point equally spaced)
    "Returns delta coords in list (x,y) on radius (rad) for vectors number (resolution)"
    step = (radians(180)/(resolution/2))
    #print(f"Fetching coords on circle for {rad = }, {resolution = }")
    #maxes= []
    list = []
    for num in range(round(radians(180)/step)+1):
        #print(f"appending for {num}")
        x,y =  ceil(rad*cos(num*step)),  ceil(rad*sin(num*step))   
        list.append((x,y))
        if y > 0:  
            list.append((x,-y))
        
   
    #print(f"Got coords on circle! {list}")
    return list

#dCoordsInRad(5)
#dCoordsOnCircle(5)

