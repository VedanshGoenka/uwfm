import math
import numpy as np


class YMDSimulation:
    '''Class to represent YMD simulation parameters'''

    def __init__(self, params):
        # Store all parameters in the same units as the config
        self.velocity = params['velocity']

        self.beta_min = params['beta_min']
        self.beta_max = params['beta_max']
        self.beta_increment = params['beta_increment']

        self.delta_min = params['delta_min']
        self.delta_max = params['delta_max']
        self.delta_increment = params['delta_increment']

        self.tolerance = params['tolerance']

    @property
    def velocity(self):
        ''' Returns the forward velocity. Default unit is [m/s]'''
        return self.__velocity/3.6

    @velocity.setter
    def velocity(self, velocity):
        self.__velocity = velocity

    @property
    def beta_min(self):
        return self.__beta_min

    @beta_min.setter
    def beta_min(self, beta_min):
        self.__beta_min = beta_min

    @property
    def beta_max(self):
        return self.__beta_max

    @beta_max.setter
    def beta_max(self, beta_max):
        self.__beta_max = beta_max

    @property
    def beta_increment(self):
        return self.__beta_increment

    @beta_increment.setter
    def beta_increment(self, beta_increment):
        self.__beta_increment = beta_increment

    @property
    def delta_min(self):
        return self.__delta_min

    @delta_min.setter
    def delta_min(self, delta_min):
        self.__delta_min = delta_min

    @property
    def delta_max(self):
        return self.__delta_max

    @delta_max.setter
    def delta_max(self, delta_max):
        self.__delta_max = delta_max

    @property
    def delta_increment(self):
        return self.__delta_increment

    @delta_increment.setter
    def delta_increment(self, delta_increment):
        self.__delta_increment = delta_increment

    @property
    def tolerance(self):
        return self.__tolerance

    @tolerance.setter
    def tolerance(self, tolerance):
        self.__tolerance = tolerance

    @property
    def beta_range(self):
        value_range = np.arange(self.beta_min, self.beta_max, self.beta_increment)
        return [math.radians(value) for value in value_range]

    @property
    def delta_range(self):
        value_range = np.arange(self.delta_min, self.delta_max, self.delta_increment)
        return [math.radians(value) for value in value_range]
