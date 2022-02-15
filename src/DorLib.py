#!/usr/bin/env python3
from numpy import linalg, array
from math import sin, cos, radians,ceil
                    
class Dorvect:    
    def __init__(self,vect = [0,0,0]):
        self.x,self.y,self.th = vect
        self.vect = array([self.x,self.y,self.th])
        self.imag = self.x + 1j*self.y
    def updateFromImag(self,imag):
        self = Dorvect([imag.real,imag.imag,self.th])
    def dist(self):
        return linalg.norm(self.vect)
    def normalized3d(self):
        norm = linalg.norm(self.vect)
        return Dorvect([val/norm for val in self.vect])
    #def toImag(self):
        #self.imag = self.x + 1j*self.y
        #return self.imag
    #def norm2d(self):
        #return max([abs(self.x),abs(self.y)])
    #def norm3d(self):
        #return max([abs(self.x),abs(self.y),abs(self.th)])   
    #def max2d(self):
       # return max([self.x,self.y])
    #def max3d(self):
        #return max([self.x,self.y,self.th])
    def __abs__(self):
         return Dorvect([abs(self.x),abs(self.y),abs(self.th)])
    def __str__(self):
        return f"Dor vector: {self.vect}"
    def __add__(self,other):
        return Dorvect([self.x + other.x, self.y + other.y,self.th + other.th])
    def __radd__(self,other):
        return Dorvect([self.x + other.x, self.y + other.y,self.th + other.th])
    def __sub__(self, other):
        return Dorvect([self.x - other.x, self.y - other.y,self.th - other.th])
    def __mul__(self, other):
        return Dorvect([self.x * other.x, self.y * other.y,self.th * other.th])
    def __div__(self, other):
        return Dorvect([self.x/ other.x, self.y / other.y,self.th/ other.th])
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
   