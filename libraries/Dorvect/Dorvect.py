class Dorvect():    
    def __init__(self,vect = [0,0,0]):
        self.x = vect[0];self.y = vect[1]; self.th = vect[2]
        self.vect = [self.x,self.y,self.th]
        self.imag = self.x + 1j*self.y
    def updateFromImag(self,imag):
        self = Dorvect([imag.real,imag.imag,self.th])
    def dist(self):
        return pow(self.x**2+self.y**2,0.5)
    #def toImag(self):
        #self.imag = self.x + 1j*self.y
        #return self.imag
    def norm2d(self):
        return max([abs(self.x),abs(self.y)])
    def norm3d(self):
        return max([abs(self.x),abs(self.y),abs(self.th)])
    def normalized3d(self):
        return Dorvect([val/self.norm3d() for val in self.vect])
    #def max2d(self):
       # return max([self.x,self.y])
    #def max3d(self):
        #return max([self.x,self.y,self.th])
    def __abs__(self):
        return Dorvect([abs(self.x),abs(self.y),abs(self.th)])
    def __str__(self):
        return f"Dor vector {self.vect}"
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