import math
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
import warnings

GRAVITY_ACCEL = 9.81  # [m/s]


class YMDParameters:
    '''Class to represent YMD simulation parameters'''

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


class YMDSolver:
    '''Helper class to deal with YMD Solving'''
    def __init__(self, vehicle, parameters):
        self.vehicle = vehicle
        self.parameters = parameters

    def converge_lateral_acceleration(self, car, beta, delta):
        '''Iterates vehicle model until the lateral acceleration converges'''

        # Initialize variables
        a_lat = 0
        a_lat_prev = 100

        # Iterate until we are within the allowed tolerance
        while math.fabs(a_lat - a_lat_prev) > self.parameters.tolerance:
            # Relaxation to allow convergence at low speeds
            a_lat = a_lat * self.parameters.relaxation + a_lat_prev * (1 - self.parameters.relaxation)
            a_lat_prev = a_lat

            # Calculate vehicle yaw speed
            yaw_speed = a_lat / self.parameters.velocity

            solution = car.calc_vehicle_forces(self.parameters.velocity, yaw_speed, a_lat, beta, delta)

            a_lat = solution.flat[1] / car.mass

        return solution

    def generate_ymd(self):
        '''Generates a Yaw Moment Diagram using the given parameters in the config files'''

        # Initialize results arraw
        result_ax = np.empty([len(self.parameters.beta_range), len(self.parameters.delta_range)])
        result_ay = np.empty([len(self.parameters.beta_range), len(self.parameters.delta_range)])
        result_mz = np.empty([len(self.parameters.beta_range), len(self.parameters.delta_range)])

        # Sweep through all the combinations of beta and delta and solve
        for i, beta in enumerate(self.parameters.beta_range):
            for j, delta in enumerate(self.parameters.delta_range):
                solution = self.converge_lateral_acceleration(self.vehicle, beta, delta)
                result_ax[i][j] = solution.flat[0]
                result_ay[i][j] = solution.flat[1]
                result_mz[i][j] = solution.flat[2]

        return result_ax, result_ay, result_mz


class YMDSimulation:
    '''Class to represent YMD simulation instance and the simulation results'''

    def __init__(self, vehicle, parameters):
        self._is_simulated = False
        self.vehicle = vehicle
        self.parameters = parameters

    @property
    def is_simulated(self):
        return self._is_simulated

    @property
    def vehicle(self):
        return self.__vehicle

    @vehicle.setter
    def vehicle(self, vehicle):
        if self._is_simulated is False:
            self.__vehicle = vehicle
        else:
            warnings.warn('Cannot assign new vehicle - simulation already completed', UserWarning)

    @property
    def ymd_parameters(self):
        return self.__vehicle

    @ymd_parameters.setter
    def ymd_parameters(self, ymd_parameters):
        if self._is_simulated is False:
            self.__ymd_parameters = ymd_parameters
        else:
            warnings.warn('Cannot assign new simulation parameters - simulation already completed', UserWarning)

    @property
    def result_ay(self):
        # Will not exist if start_simulation() is not ran.
        return self._result_ay

    @property
    def result_ax(self):
        # Will not exist if start_simulation() is not ran.
        return self._result_ax

    @property
    def result_mz(self):
        # Will not exist if start_simulation() is not ran.
        return self._result_mz

    @property
    def nondim_result_ay(self):
        return self._result_ay / (self.car.mass * GRAVITY_ACCEL)

    @property
    def nondim_result_ax(self):
        return self._result_ax / (self.car.mass * GRAVITY_ACCEL)

    @property
    def nondim_result_mz(self):
        return self._result_mz / (self.car.mass * GRAVITY_ACCEL * self.car.wheelbase)

    def start_simulation(self):
        ''' Run the simulation with the help of the YMDSolver object '''
        solver = YMDSolver(self.vehicle, self.parameters)
        self._result_ay, self._result_ax, self._result_mz = solver.generate_ymd()

        # Raise a flag once the simulation has started
        self._is_simulated = True


class YMDAnalysis:
    '''Helper class for post-processing and analysis of a YMDSimulation
    class'''

    def __init__(self, result):
        self.simulation = result

    @property
    def simulation(self):
        return self.__simulation

    @simulation.setter
    def simulation(self, result):
        if result.is_simulated is True:
            self.__simulation = result
        else:
            self.__simulationt = None
            warnings.warn('YMDSimulation contains no results', UserWarning)

    def plot_results(self):
        '''Plots the raw results of the simulation. Warning: no protection if variables are empty!'''
        plt.plot(np.transpose(self.simulation.result_ay)[:][:],
                 np.transpose(self.simulation.result_mz)[:][:], color='black')
        plt.plot(self.simulation.result_ay[:][:], self.simulation.result_mz[:][:], color='red')

        plt.xlabel('Lateral Acceleration [m/s]')
        plt.ylabel('Yaw Moment [Nm]')
        plt.title('Yaw Moment Diagram - {:.1f} [m/s]'.format(self.simulation.parameters.velocity))
        plt.grid(True)

        const_steer = mpatches.Patch(color='red', label='Constant Steer')
        const_slip = mpatches.Patch(color='black', label='Constant Vehicle Slip')

        # Define the legend and the legend font size
        plt.legend(handles=[const_steer, const_slip], prop={'size': 12})

        plt.show()

    def plot_nondim_results(self):
        '''Plots the nondimensionalized results of the simulation. Warning: no protection if variables are empty!'''
        plt.plot(np.transpose(self.simulation.nondim_result_ay)[:][:],
                 np.transpose(self.simulation.nondim_result_mz)[:][:], color='black')
        plt.plot(self.simulation.nondim_result_ay[:][:],
                 self.simulation.nondim_result_mz[:][:], color='red')

        plt.xlabel('Lateral Acceleration [m/s]')
        plt.ylabel('Yaw Moment [Nm]')
        plt.title('Yaw Moment Diagram - {:.1f} [m/s]'.format(self.simulation.parameters.velocity))
        plt.grid(True)

        const_steer = mpatches.Patch(color='red', label='Constant Steer')
        const_slip = mpatches.Patch(color='black', label='Constant Vehicle Slip')

        # Define the legend and the legend font size
        plt.legend(handles=[const_steer, const_slip], prop={'size': 12})

        plt.show()
