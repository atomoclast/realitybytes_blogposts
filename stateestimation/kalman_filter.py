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
import matplotlib.pyplot as plt
import pprint


def kalman_filter(x, P, measurement, R, u, Q, F, H):
    """
        Implements a basic Kalman Filter Algorithm algorithm
        Input:
        x - initial state, [x1, x2, x0_dot, x1_dot]
        P - Covariance matrix, initial uncertainty
        measurement, observed position
        R - Measurement Noise/Uncertainty.
        u - external motion
        Q - Motion Noise
        F - Next State Function
        H - Measurement Function
        """


    # Update:

    y = np.matrix(measurement).transpose() - H * x
    S = H * P * H.transpose() + R  # residual convariance
    K_t = P * H.transpose() * S.I  # Kalman gain
    x = x + K_t*y  #state update estimate
    I = np.matrix(np.eye(F.shape[0])) # identity matrix
    P = (I - K_t*H)*P

    # Predict:
    x = F*x + u
    P = F*P*F.transpose() + Q

    return x, P

if __name__ == "__main__":
    x = np.matrix([0, 0, 0, 0]).transpose()  # Initial state, at (0,0), at rest.
    P = np.matrix(np.eye(4))*1000  # initial uncertainty
    F = np.matrix([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]])
    H = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0]])
    u = np.matrix([[0, 0, 0, 0]]).transpose()
    Q = np.eye(4)
    R = 0.01 ** 2

    N = 20
    true_x = np.linspace(0.0, 50.0, N)
    true_y = true_x*2
    sensed_x = true_x + 0.05*np.random.random(N)*true_x
    sensed_y = true_y + 0.05*np.random.random(N)*true_y
    plt.plot(sensed_x, sensed_y, 'ro', label = 'Noisy Measurements.')
    result = []
    for measurements in zip(sensed_x, sensed_y):
        x, P = kalman_filter(x, P, measurements, R, u, Q, F, H)
        result.append((x[:2]).tolist())
    kalman_x, kalman_y = zip(*result)
    plt.plot(kalman_x, kalman_y, 'b-', label='Kalman Estimate')
    plt.title("Kalman Filtering")
    plt.legend(loc='upper left')
    plt.show()

print "Plotted"

