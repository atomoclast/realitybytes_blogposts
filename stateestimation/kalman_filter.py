#!/usr/bin/python

'''
BSD 2-Clause License

Copyright (c) 2017, Andrew Dahdouh
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

import numpy as np

def kalman_filter(x, P):
    for n in range(len(measurements)):
        #measurement update
        error = np.array(measurements[n]) - H * x
        print "error: ", error

        residual_covar = H * P * H.transpose() + R
        print "residual covar: ", residual_covar

        inv_residual_covar = np.linalg.inv(residual_covar)
        kgain = P * H.transpose() * inv_residual_covar #kalman gain
        print "Kalman Gain: ", kgain

        x = x + kgain * error
        print "x measure: ", x

        P = (np.eye(2) - kgain * H) * P
        print "P measure: ", P

        # prediction
        x = F * x + u
        print "x predict: ", x

        P = F * P * F.transpose()
        print "P predict: ", P

    return x, P




#Test Main: 
measurements = np.array([1, 2, 3])

#Initial State [location, velocity], 2x1?
x = np.matrix([[0], [0]])
P = np.matrix([[1000., 0.], [0., 1000.]])  # initial uncertainty
u = np.matrix([[0.], [0.]])  # external motion
F = np.matrix([[1., 1.], [0, 1.]])  # next state function
H = np.matrix([[1., 0.]])  # measurement function
R = np.matrix([[1.]])  # measurement uncertainty

print kalman_filter(x, P)
# output should be:
# x: [[3.9996664447958645], [0.9999998335552873]]
# P: [[2.3318904241194827, 0.9991676099921091], [0.9991676099921067, 0.49950058263974184]]
