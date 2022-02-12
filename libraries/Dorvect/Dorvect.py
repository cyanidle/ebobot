import numpy as np
class Dorvect(np.array):    
    def __init__(self,vect = [0,0,0]):
        self.x,self.y,self.th = vect
        self.vect = np.array([self.x,self.y,self.th])
        self.imag = self.x + 1j*self.y
    def updateFromImag(self,imag):
        self = Dorvect([imag.real,imag.imag,self.th])
    def dist(self):
        return np.linalg.norm(self.vect)
    def normalized3d(self):
        norm = np.linalg.norm(self.vect)
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
    # def __abs__(self):
    #     return Dorvect([abs(self.x),abs(self.y),abs(self.th)])
    # def __str__(self):
    #     return f"Dor vector: {self.vect}"
    # def __add__(self,other):
    #     return Dorvect([self.x + other.x, self.y + other.y,self.th + other.th])
    # def __radd__(self,other):
    #     return Dorvect([self.x + other.x, self.y + other.y,self.th + other.th])
    # def __sub__(self, other):
    #     return Dorvect([self.x - other.x, self.y - other.y,self.th - other.th])
    # def __mul__(self, other):
    #     return Dorvect([self.x * other.x, self.y * other.y,self.th * other.th])
    # def __div__(self, other):
    #     return Dorvect([self.x/ other.x, self.y / other.y,self.th/ other.th])