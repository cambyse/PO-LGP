import random
import math
import numpy as np


class Kalman(object):


    def __init__(self, B, x, P, R):
        self.B = B                      # Control matrix
        self.x = x                      # Initial state estimate.
        self.P = P                      # Initial covariance estimate.
        self.R = R                      # Estimated error in measurements.


    def step(self, A, H, u, M):
        # M     measure
        # u     motion 
        # H     Observation matrix
        # A     Transition matrix

        # Predict
        xe = (A * self.x + self.B * u)
        Pe = A * self.P * A.T

        # Observe
        y = M - H * xe # error

        # Compute Kalman gain
        S = H * Pe * H.T + self.R 
        kg = Pe * H.T * np.linalg.inv(S)

        # Update state
        self.x = xe + kg * y

        # Update covariance estimate
        size = self.P.shape[0]
        self.P = (np.eye(size)-kg*H)*Pe

        return self.x


    def state(self):
        return self.x


class Lowpass(object):

    h = {}

    @staticmethod
    def filter(key, value, num=21):
        if key not in Lowpass.h:
            Lowpass.h[key] = []
        Lowpass.h[key] = Lowpass.h[key][1-num:]
        Lowpass.h[key].append(value)

        return np.mean(Lowpass.h[key], axis=0)


