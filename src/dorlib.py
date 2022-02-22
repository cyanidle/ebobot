#!/usr/bin/env python3
#from numpy import linalg, array

from math import sin, cos, radians,ceil           
def dCoordsInRad(rad,resolution = 3):  
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


 
def dCoordsOnCircle(rad,resolution = 5): #in cells
    step = int(radians(180)//resolution)
    print(f"Fetching coords on circle for rad = {rad}, res = {resolution}")
    #maxes= []
    list = []
    for num in range(0,round(radians(180))+1,step):
        #print(f"appending for {num}")
        x,y =  round(rad*cos(num)),  round(rad*sin(num))   
        list.append((x,y))
        if y > 0:  
            list.append((x,-y))
        if x > 0:
            list.append((-x,y))
            if y > 0: 
                list.append((-x,-y))
   
    print(f"Got coords on circle! {list}")
    return list

#dCoordsInRad(5)
dCoordsOnCircle(5)