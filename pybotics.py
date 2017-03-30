
import math, inspect
import numpy as np
########################rotation matrices
#######################
def rot2(phi,*flag):
# create a 2d rotation  matrix given the angles in radians or degree
# flag==deg computes the rotation matrix in degrees
 #   t=inspect.getargspec(Rmatrix)
#    print t.args
    if len(flag) > 0:
        phi = (phi*math.pi)/180

    Rma=np.array([[math.cos(phi), -math.sin(phi)],[math.sin(phi), math.cos(phi)]])
    return np.round(Rma,4)



def trot2(phi,*flag):
# create a 2d rotation  matrix given the angles in radians or degree
# flag==deg computes the rotation matrix in degrees
 #   t=inspect.getargspec(Rmatrix)
#    print t.args
    if len(flag) > 0:
        phi = (phi*math.pi)/180

    Rhma=np.array([[math.cos(phi), -math.sin(phi), 0],[math.sin(phi), math.cos(phi), 0],[0, 0 , 1]])
    return np.round(Rhma,4)

###################translation matrices##################
#######################################################
def trans12(x,y):
# create a translation matrix given  x and y in homogenus coordinates
# flag==deg computes the rotation matrix in degrees
 #   t=inspect.getargspec(Rmatrix)
#    print t.args
    Tma=np.array([[1, 0 , x],[0, 1, y], [0,0,1]])
    return np.round(Tma,4)

def se2(phi,x,y,*flag):
# create a rotation and translation matrix given the angles in radians or degree
# flag==deg computes the rotation matrix in degrees
 #   t=inspect.getargspec(Rmatrix)
#    print t.args

    if len(flag) > 0:
        phi = (phi*math.pi)/180

    Tma=np.array([[math.cos(phi), -math.sin(phi), x],[math.sin(phi), math.cos(phi), y], [0,0,1]])
    return np.round(Tma,4)

#######################################################
################rotation in 3d
#######################################################
def rotx(phi,*flag):
# create a rotation  matrix in the x (pitch) axes in radians or degree
# flag==deg computes the rotation matrix in degrees
 #   t=inspect.getargspec(Rmatrix)
#    print t.args
    if len(flag) > 0:
        phi = (phi*math.pi)/180

    Rx=np.array([ [1, 0, 0], [0, math.cos(phi), -math.sin(phi)],[0, math.sin(phi), math.cos(phi)]])
    return np.round(Rx,4)

def roty(phi,*flag):
# create a rotation  matrix in the y (roll) axes in radians or degree
# flag==deg computes the rotation matrix in degrees
 #   t=inspect.getargspec(Rmatrix)
#    print t.args
    if len(flag) > 0:
        phi = (phi*math.pi)/180

    Ry=np.array([[math.cos(phi),0,  math.sin(phi)],[0, 1, 0],[-math.sin(phi), 0,  math.cos(phi)]])
    return np.round(Ry,4)

def rotz(phi,*flag):
# create a rotation  matrix in the z (yaw) axes in radians or degree
# flag==deg computes the rotation matrix in degrees
 #   t=inspect.getargspec(Rmatrix)
#    print t.args
    return trot2(phi,*flag)


def eul2r(x,y,z,*flag):
## rotation matrix given the Euler angles
    return np.round(np.dot(np.dot(rotz(x,*flag),roty(y,*flag)),rotz(z,*flag)),4)

def heul2r(x,y,z,*flag):
## rotation matrix given the Euler angles
    R = np.round(np.dot(np.dot(rotz(x,*flag),roty(y,*flag)),rotz(z,*flag)),4)
    L =  np.array([[0],[0],[0]])
    L2=  np.array([[0,0,0,1]])
    return np.vstack((np.hstack((R,L)),L2))

def rpy2r(x,y,z,*flag):
##Roll pich yaw angles, given the
    return np.round(np.dot(np.dot(rotx(x,*flag),roty(y,*flag)),rotz(z,*flag)),4)

def hrpy2r(x,y,z,*flag):
##Roll pich yaw angles, given the
    return np.round(np.dot(np.dot(rotx(x,*flag),roty(y,*flag)),rotz(z,*flag)),4)

###################Quaternions
##################################################

def Quaternion(R):
    def SIGN(x):
        if x >= 0:
            return 1.0
        else:
            return -1.0

    def NORM(a, b, c, d):
        return math.sqrt(a * a + b * b + c * c + d * d)
    r11=R[0,0]
    r12=R[0,1]
    r13=R[0,2]
    r21=R[1,0]
    r22=R[1,1]
    r23=R[1,2]
    r31=R[2,0]
    r32=R[2,1]
    r33=R[2,2]
    q0 = ( r11 + r22 + r33 + 1) / 4
    q1 = ( r11 - r22 - r33 + 1) / 4
    q2 = (-r11 + r22 - r33 + 1) / 4
    q3 = (-r11 - r22 + r33 + 1) / 4
    if(q0 < 0):
         q0 = 0
    if(q1 < 0):
         q1 = 0
    if(q2 < 0):
         q2 = 0
    if(q3 < 0):
        q3 = 0
    q0 = math.sqrt(q0)
    q1 = math.sqrt(q1)
    q2 = math.sqrt(q2)
    q3 = math.sqrt(q3)
    if(q0 >= q1 and q0 >= q2 and q0 >= q3):
        q0 *= +1
        q1 *= SIGN(r32 - r23)
        q2 *= SIGN(r13 - r31)
        q3 *= SIGN(r21 - r12)
    elif(q1 >= q0 and q1 >= q2 and q1 >= q3):
        q0 *= SIGN(r32 - r23)
        q1 *= +1
        q2 *= SIGN(r21 + r12)
        q3 *= SIGN(r13 + r31)
    elif(q2 >= q0 and q2 >= q1 and q2 >= q3):
        q0 *= SIGN(r13 - r31)
        q1 *= SIGN(r21 + r12)
        q2 *= 1
        q3 *= SIGN(r32 + r23)
    elif(q3 >= q0 and q3 >= q1 and q3 >= q2):
        q0 *= SIGN(r21 - r12)
        q1 *= SIGN(r31 + r13)
        q2 *= SIGN(r32 + r23)
        q3 *= +1
    else:
        print "coding error\n"

    r = NORM(q0, q1, q2, q3)
    q0 /= r
    q1 /= r
    q2 /= r
    q3 /= r
    Quad=np.array([round(q0,4), round(q1,4), round(q2,4), round(q3,4)])
     #round(q0,4), round(q1,4), round(q2,4), round(q3,4)
    return Quad
