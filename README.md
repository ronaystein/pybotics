# pybotics
This repository contains a set of functions for robotics

#Requirements
python 2.7X with numpy and math libraries
just copy in your working directory
############rotation matrices##############
#example
rot2: create a 2d rotation  matrix given the angles in radians or degree  

#for example
import pybotics as pb
import numpy as np

M= pb.rotz(45,'deg')
print M

##will produce

  [[ 0.7071 -0.7071  0.    ] \n
  [ 0.7071  0.7071  0.    ] \n
  [ 0.      0.      1.    ]]\n

##the angles can be expressed also in radians
M= pb.rotz(45,'rad')
print M

#with the same result

####################
# list of functions
###################

trot2 : 2d rotation matrix in homogenus form
trans12: create a translation matrix given  x and y in homogenus coordinates
se2:   create a rotation and translation matrix given the angles in radians or degree


#######################################################
################rotation in 3d
#######################################################

rotx:   create a rotation  matrix in the x (pitch) axes in radians or degree
roty:   create a rotation  matrix in the y (roll) axes in radians or degree
rotz:   create a rotation  matrix in the z (yaw) axes in radians or degree
eul2r:  rotation matrix given the Euler angles
heul2r: rotation matrix given the Euler angles  in homogenus form
rpy2r:  Roll pich yaw angles, given the angles in x,y, and z
Quaternion:  quaternions


