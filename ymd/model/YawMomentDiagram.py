import math
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
import warnings

from scipy.interpolate import interp1d
from scipy.optimize import fsolve

np.seterr(divide='raise')

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
        '''while math.fabs(a_lat - a_lat_prev) > self.parameters.tolerance:
            # Relaxation to allow convergence at low speeds
            a_lat = a_lat * self.parameters.relaxation + a_lat_prev * (1 - self.parameters.relaxation)
            a_lat_prev = a_lat

            # Calculate vehicle yaw speed
            yaw_speed = a_lat / self.parameters.velocity

            solution = car.calc_vehicle_forces(self.parameters.velocity, yaw_speed, a_lat, beta, delta)

            a_lat = solution.flat[1] / car.mass
        '''

        vehicle_accel = np.array([0, 0, 0])
        vehicle_accel_prev = np.array([100, 100, 100])
        first_iteration_flag = True

        while math.fabs(np.linalg.norm(vehicle_accel) - np.linalg.norm(vehicle_accel_prev)) > self.parameters.tolerance:
            # Apply a relaxation factor each interation to slow down
            # simulation. This is for solver convergence.
            if first_iteration_flag is False:
                vehicle_accel = vehicle_accel * self.parameters.relaxation + vehicle_accel_prev * (1 - self.parameters.relaxation)
            else:
                first_iteration_flag = False
            
            # Keep track of the vehicle acceleration from the previous iteration
            vehicle_accel_prev = vehicle_accel

            # Define rotation matrix from body-fixed vehicle frame to
            # normal-tangential coordinates
            rotation_matrix = np.array([[math.cos(beta) , math.sin(beta), 0],
                                        [-math.sin(beta), math.cos(beta), 0],
                                        [              0,              0, 1]])

            # Normal unit vector
            unit_vector_normal = np.dot(rotation_matrix.transpose(), np.array([0, 1, 0]))

            # Rotate vehicle accelerations to normal-tangential frame and find
            # the magnitude of the rotation arm, rho
            rotated_vehicle_accel = np.dot(rotation_matrix, vehicle_accel)
            try:
                rho = self.parameters.velocity**2 / rotated_vehicle_accel[1]
            except FloatingPointError:
                rho = 0

            # Package the location of the yaw centre with respect to the
            # vehicle centre-of-gravity.
            yaw_centre = rho * unit_vector_normal

            # Calculate the vehicle yaw speed
            yaw_speed = rotated_vehicle_accel[1] / self.parameters.velocity

            # Calculate the vehicle forces and normalize it by the vehicle mass.
            # Ignore the yaw moment since we don't apply anything to it
            vehicle_forces = car.calc_vehicle_forces(self.parameters.velocity, yaw_speed, yaw_centre, vehicle_accel[1], beta, delta)
            vehicle_accel = vehicle_forces / car.mass

        return vehicle_forces

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
        return self._result_ay / (self.vehicle.mass)

    @property
    def result_ax(self):
        # Will not exist if start_simulation() is not ran.
        return self._result_ax / (self.vehicle.mass)

    @property
    def result_mz(self):
        # Will not exist if start_simulation() is not ran.
        return self._result_mz

    @property
    def nondim_result_ay(self):
        return self._result_ay / (self.vehicle.mass * GRAVITY_ACCEL)

    @property
    def nondim_result_ax(self):
        return self._result_ax / (self.vehicle.mass * GRAVITY_ACCEL)

    @property
    def nondim_result_mz(self):
        return self._result_mz / (self.vehicle.mass * GRAVITY_ACCEL * self.vehicle.wheelbase)

    def start_simulation(self):
        ''' Run the simulation with the help of the YMDSolver object '''
        solver = YMDSolver(self.vehicle, self.parameters)
        self._result_ax, self._result_ay, self._result_mz = solver.generate_ymd()

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

    @property
    def max_lateral_accel(self):
        '''Finds the maximum lateral acceleration in the yaw moment diagram in [g]'''
        i, j = self._max_lateral_accel_idx()

        return self.simulation.result_ay[i][j] / GRAVITY_ACCEL

    @property
    def residual_yaw_moment(self):
        '''Finds the residual yaw moment at maximum lateral acceleration in [Nm]'''
        i, j = self._max_lateral_accel_idx()

        return self.simulation.result_mz[i][j]

    @property
    def trim_lateral_accel(self):
        '''Finds the maximum trim lateral acceleration in [g]'''

        max_trim_latacc = 0

        # For each vehicle slip isoline
        for i, beta in enumerate(self.simulation.parameters.beta_range):
            # Dectect when the lateral acceleration is no longer decreasing
            deriv_isoline = np.gradient(self.simulation.result_ay[i,:])
            range_of_isoline = np.nonzero(deriv_isoline < 0) 

            # Interpolate within the range of interest
            try:
                interped = interp1d(self.simulation.result_mz[i, range_of_isoline].squeeze(),
                                    self.simulation.result_ay[i, range_of_isoline].squeeze(),
                                    kind='linear')

                # If there is an error, ignore the result
                trim_latacc = interped(0)
            except ValueError:
                trim_latacc = 0

            # Check if we have a new candidate for trim latacc
            if trim_latacc > max_trim_latacc:
                max_trim_latacc = trim_latacc

        # Now for each steer isoline
        for j, delta in enumerate(self.simulation.parameters.delta_range):
            deriv_isoline = np.gradient(self.simulation.result_ay[:,j])
            range_of_isoline = np.nonzero(deriv_isoline < 0) 

            try:
                interped = interp1d(self.simulation.result_mz[range_of_isoline, j].squeeze(),
                                    self.simulation.result_ay[range_of_isoline, j].squeeze(),
                                    kind='linear')

                trim_latacc = interped(0)
            except ValueError:
                trim_latacc = 0

            if trim_latacc > max_trim_latacc:
                max_trim_latacc = trim_latacc

        return max_trim_latacc / GRAVITY_ACCEL

    def _max_lateral_accel_idx(self):
        i, j = np.unravel_index(self.simulation.result_ay.argmax(),
                                self.simulation.result_ay.shape)
        
        return i, j

    @property
    def entry_control(self):
        # Find the index where beta and delta are zero
        beta_idx, delta_idx = self._zero_zero_idx() 

        # Calculate the change in yaw moment
        delta_mz = self.simulation.result_mz[beta_idx,delta_idx-1] - self.simulation.result_mz[beta_idx,delta_idx]

        # Calculate the control 
        return delta_mz / self.simulation.parameters.delta_increment

    @property
    def entry_stability(self):
        # Find the index where beta and delta are zero
        beta_idx, delta_idx = self._zero_zero_idx() 

        # Calculate the change in yaw moment
        delta_mz = self.simulation.result_mz[beta_idx-1,delta_idx] - self.simulation.result_mz[beta_idx,delta_idx]

        # Calculate the control 
        return delta_mz / self.simulation.parameters.beta_increment

    def _zero_zero_idx(self):
        beta_idx = self.simulation.parameters.beta_range.index(0)
        delta_idx = self.simulation.parameters.delta_range.index(0)

        return beta_idx, delta_idx

    def plot_results(self):
        '''Plots the raw results of the simulation. Warning: no protection if variables are empty!'''
        plt.figure()
        plt.plot(np.transpose(self.simulation.nondim_result_ay)[:][:],
                 np.transpose(self.simulation.result_mz)[:][:], color='black')
        plt.plot(self.simulation.nondim_result_ay[:][:], self.simulation.result_mz[:][:], color='red')

        plt.xlabel('Ay - Lateral Acceleration [g]')
        plt.ylabel('Mz - Yaw Moment [Nm]')
        plt.title('Yaw Moment Diagram - {:.1f} [m/s]'.format(self.simulation.parameters.velocity))
        plt.grid(True)

        const_steer = mpatches.Patch(color='red', label='Constant Steer')
        const_slip = mpatches.Patch(color='black', label='Constant Vehicle Slip')

        # Define the legend and the legend font size
        plt.legend(handles=[const_steer, const_slip], prop={'size': 12})

    def plot_nondim_results(self):
        '''Plots the nondimensionalized results of the simulation. Warning: no protection if variables are empty!'''
        plt.figure()
        plt.plot(np.transpose(self.simulation.nondim_result_ay)[:][:],
                 np.transpose(self.simulation.nondim_result_mz)[:][:], color='black')
        plt.plot(self.simulation.nondim_result_ay[:][:],
                 self.simulation.nondim_result_mz[:][:], color='red')

        plt.xlabel('Ay - Lateral Acceleration [g]')
        plt.ylabel('Cn - Non-Dimensionalized Yaw Moment')
        plt.title('Yaw Moment Diagram - {:.1f} [m/s]'.format(self.simulation.parameters.velocity))
        plt.grid(True)

        const_steer = mpatches.Patch(color='red', label='Constant Steer')
        const_slip = mpatches.Patch(color='black', label='Constant Vehicle Slip')

        # Define the legend and the legend font size
        plt.legend(handles=[const_steer, const_slip], prop={'size': 12})

    def show_plots(self):
        plt.show()
