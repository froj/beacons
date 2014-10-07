
from math import sqrt,cos,sin,atan2,pi

class Vec2D(object):

    EPSILON = 0.0001

    def __init__(self,x,y):
        self.x = x
        self.y = y

    def dot(self,other):
        return self.x * other.x + self.y * other.y

    def cross(self,other):
        return (self.x * other.y - self.y * other.x)

    def length(self):
        return sqrt(self.dot(self))

    def normalized(self):
        d = self.length()
        return self * (1/d)

    def rotate(self,angle,translation):

        centered = self - translation

        cs = cos(angle)
        sn = sin(angle) 

        nX = cs * centered.x - sn * centered.y
        nY = sn * centered.x + cs * centered.y

        final = Vec2D(nX,nY) + translation

        return final

    def directed_angle(self, other):
        angle = atan2(other.y, other.x) - atan2(self.y, self.x)

        if angle < 0:
            return angle + 2 * pi
        else:
            return angle

    def __neg__(self):
        return Vec2D(-self.x,-self.y)

    def __add__(self,other):
        return Vec2D(self.x + other.x,self.y + other.y)

    def __sub__(self,other):
        return Vec2D(self.x - other.x,self.y - other.y)

    def __mul__(self,other):
        return Vec2D(self.x * other, self.y * other)

    def __str__(self):
        return "({x},{y})".format(x=self.x,y=self.y)

    def __eq__(self,other):
        return ((self - other).length() < Vec2D.EPSILON)

    def __ne__(self,other):
        return not self == other

def orientation(v1,v2,v3):
    v12 = v2 - v1
    v23 = v3 - v2
    return v12.cross(v23)
