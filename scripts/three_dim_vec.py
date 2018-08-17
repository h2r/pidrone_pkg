import numpy as np

class ThreeDimVec(object):
    ''' Struct to store three variables referenced as x,y,z'''
    # This should be turned into a data class with the next python 3 release
    def __init__(self, x=0.0, y=0.0 ,z=0.0):
        self.x = x
        self.y = y
        self.z = z
    def __str__(self):
        return "x: %f, y: %f, z: %f" % (self.x, self.y, self.z)

    def __mul__(self, other):
        return ThreeDimVec(self.x * other, self.y * other, self.z * other)

    def __rmul__(self, other):
        return self.__mul__(other)

    def __div__(self, other):
        return ThreeDimVec(self.x / other, self.y / other, self.z / other)

    def __add__(self, other):
        return ThreeDimVec(self.x + other.x, self.y + other.y, self.z + other.z)

    def __radd__(self, other):
        return self.__add__(other)

    def __sub__(self, other):
        return ThreeDimVec(self.x - other.x, self.y - other.y, self.z - other.z)

    def magnitude(self):
        return np.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def planar_magnitude(self):
        return np.sqrt(self.x * self.x + self.y * self.y)


# TODO EDIT ALL DATA DESCRIPTIONS
class Position(ThreeDimVec):
    ''' Struct to store postion components x,y,yaw
    x : the x position
    y : the y position
    yaw : the yaw value
    '''
    def __init__(self, x=0.0, y=0.0, z=0.0):
        super(Position, self).__init__(x,y,z)

class Velocity(ThreeDimVec):
    ''' Struct to store velocity components vx,vy,vz
    vx : the x velocity
    vy : the y velocity
    vz : the z velocity
    '''
    def __init__(self, x=0.0, y=0.0, z=0.0):
        super(Velocity, self).__init__(x,y,z)

class Error(ThreeDimVec):
    ''' Struct to store error which is in the form vx,vy,z
    x : the x velocity error
    y : the y velocity error
    z  : the z position error
    '''
    def __init__(self, x=0.0, y=0.0, z=0.0):
        super(Error, self).__init__(x,y,z)

class RPY(ThreeDimVec):
    ''' Struct to store the roll, pitch, and yaw values which is in the form
    r : roll
    p : pitch
    y : kp_yaw
    '''
    def __init__(self, r=0.0, p=0.0, y=0.0):
        super(RPY, self).__init__(r,p,y)
        self.r = self.x
        self.p = self.y
        self.y = self.z
