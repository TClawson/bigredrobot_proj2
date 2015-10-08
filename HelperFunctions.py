#!/usr/bin/env python

import numpy

#NOTE: Pass in all transforms relative to origin O_0
def jacobian(transforms):
    #Get z vector of each frame
    #   -Each transform has a quaternion
    #   -Use the quaternion to transform a Pos into the new frame
    #   -Each transform has a translation object that gives us the origin (O_i)

    #Create a 6xn matrix

    #for i in 0 : n
    #   top three rows: z_i * (O_n - O_i)
    #   bottom three rows: z_i

    #return jacobian
