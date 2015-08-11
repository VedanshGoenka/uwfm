import math
import matplotlib.pyplot as plt
import numpy as np


class YMDSimulation:
    '''Class to represent YMD simulation instance'''

    def __init__(self, params):
        # Store all parameters in the same units as the config

        # Simulation parameters
        self.velocity = params['velocity']

        self.beta_min = params['beta_min']
        self.beta_max = params['beta_max']
        self.beta_increment = params['beta_increment']

        self.delta_min = params['delta_min']
        self.delta_max = params['delta_max']
        self.delta_increment = params['delta_increment']

        self.tolerance = params['tolerance']
        self.relaxation = params['relaxation']

        # Stashing the results
        # TODO: is this more appropriate to make another class to describe this?

        # Stash the speed
        self.result_velocity = None

        # Stash the simulation range
        self.result_betarange = None
        self.result_deltarange = None
        
        # Stash the vehicle forces
        self.result_ax = None
        self.result_ay = None
        self.result_mz = None

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
    def relaxation(self):
        return self.__relaxation

    @relaxation.setter
    def relaxation(self, relaxation):
        self.__relaxation = relaxation

    @property
    def beta_range(self):
        value_range = np.arange(self.beta_min, self.beta_max+1, self.beta_increment)
        return [math.radians(value) for value in value_range]

    @property
    def delta_range(self):
        value_range = np.arange(self.delta_min, self.delta_max+1, self.delta_increment)
        return [math.radians(value) for value in value_range]

    def plot_results(self):
        '''Plots the results of the simulation. Warning: no protection if variables are empty!'''
        plt.plot(np.transpose(self.result_ay)[:][:],
                 np.transpose(self.result_mz)[:][:], color='black')
        plt.plot(self.result_ay[:][:], self.result_mz[:][:], color='red')

        plt.xlabel('Lateral Acceleration [g]')
        plt.ylabel('Yaw Moment [Nm]')
        plt.title('Yaw Moment Diagram - {:.1f} [m/s]'.format(self.velocity))
        plt.grid(True)

        plt.show()

    def converge_lateral_acceleration(self, car, beta, delta):
        '''Iterates vehicle model until the lateral acceleration converges'''

        # Initialize variables
        a_lat = 0
        a_lat_prev = 100

        # Iterate until we are within the allowed tolerance
        while math.fabs(a_lat - a_lat_prev) > self.tolerance:
            # Relaxation to allow convergence at low speeds
            a_lat = a_lat * self.relaxation + a_lat_prev * (1 - self.relaxation)
            a_lat_prev = a_lat

            # Calculate vehicle yaw speed
            yaw_speed = a_lat / self.velocity

            solution = car.calc_vehicle_forces(self.velocity, yaw_speed, a_lat, beta, delta)

            a_lat = solution.flat[1] / car.mass

        return solution

    def generate_ymd(self, car):
        '''Generates a Yaw Moment Diagram using the given parameters in the config files'''

        # Copy over current simulaiton results into the results section
        # TODO: Figure out how you want this object to behave in terms of holdin gon to results

        # Initialize results arraw
        result_ax = np.empty([len(self.beta_range), len(self.delta_range)])
        result_ay = np.empty([len(self.beta_range), len(self.delta_range)])
        result_mz = np.empty([len(self.beta_range), len(self.delta_range)])

        # Sweep through all the combinations of beta and delta and solve
        for i, beta in enumerate(self.beta_range):
            for j, delta in enumerate(self.delta_range):
                solution = self.converge_lateral_acceleration(car, beta, delta)

                result_ax[i][j] = solution.flat[0] / car.mass
                result_ay[i][j] = solution.flat[1] / car.mass
                result_mz[i][j] = solution.flat[2]

        # Copy over the results to our class instance
        self.result_ax = result_ax
        self.result_ay = result_ay
        self.result_mz = result_mz

        return result_ax, result_ay, result_mz

