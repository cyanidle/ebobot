#!/usr/bin/env python3
#from numpy import linalg, array

from math import sin, cos, radians,ceil           
def dCoordsInRad(rad,resolution = 3):  
    step = radians(90)/resolution                     
    maxes_list = []
    list = []
    print(f"Fetching all coords in radius {rad}, with res {resolution}")
    for num in range(round(radians(90)/step) + 1):   
        x_max = ceil(sin(num*step)*rad)   
        y_max = ceil(cos(num*step)*rad)
        print(f"x_max = {x_max}, y_max = {y_max}")
        maxes_list.append((x_max,y_max))
    print(f"Done!")
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



def dCoordsOnCircle(rad,resolution = 3):
    step = radians(180)/resolution
    maxes= []
    list = []
    for num in range(round(radians(180)/step) + 1):
        maxes.append((    round(rad*sin(num*step)),round(rad*cos(num*step))    ))
    for x,y in maxes:
        list.append((x,y))
        list.append((x,-y))
    print(f"Got coords on circle! {list}")
    return list

#dCoordsInRad(5)