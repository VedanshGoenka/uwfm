import math
import numpy as np

from scipy.optimize import fsolve


class Quartet:
    '''An arbitrary class that holds the four corner values of a car.'''

    def __init__(self, fr, fl, rr, rl):
        self.quartet = [fr, fl, rr, rl]

    @property
    def quartet(self):
        return self.__quartet

    @quartet.setter
    def quartet(self, data):
        if len(data) == 4:
            self.__quartet = data
        else:
            self.__quartet = [0, 0, 0, 0]

    @property
    def fr(self):
        return self.quartet[0]

    @property
    def fl(self):
        return self.quartet[1]

    @property
    def rr(self):
        return self.quartet[2]

    @property
    def rl(self):
        return self.quartet[3]


class Vehicle:
    def __init__(self, tires, mass, geometry, suspension):
        # Tires
        self.tires = tires  # TODO: protect this data member

        # General
        self.suspended_mass = mass['suspended_mass']
        self.nonsuspended_mass = mass['nonsuspended_mass']
        self.driver_mass = mass['driver_mass']
        self.cg_height = mass['cg_height']  # CG height [m]
        self.weightdist_front = mass['weightdist_front']  # front weight dist [%]

        # Geometry
        self.wheelbase = geometry['wheelbase']  # wheelbase [m]
        self.trackwidth = geometry['trackwidth']  # track wdith [m]
        self.trackwidth_front = geometry['trackwidth_front']
        self.trackwidth_rear = geometry['trackwidth_rear']
        self.rollcentre_front = geometry['rollcentre_front']
        self.rollcentre_rear = geometry['rollcentre_rear']
        self.vsal_front = geometry['vsal_front']
        self.vsal_rear = geometry['vsal_rear']

        # Suspension
        self.cornerspring_front = suspension['cornerspring_front']
        self.cornerspring_rear = suspension['cornerspring_rear']
        self.antirollstiffness_front = suspension['antirollstiffness_front']
        self.antirollstiffness_rear = suspension['antirollstiffness_rear']

    '''Mass'''
    @property
    def suspended_mass(self):
        return self.__suspended_mass

    @suspended_mass.setter
    def suspended_mass(self, suspended_mass):
        self.__suspended_mass = suspended_mass

    @property
    def nonsuspended_mass(self):
        return self.__nonsuspended_mass + self.driver_mass

    @nonsuspended_mass.setter
    def nonsuspended_mass(self, nonsuspended_mass):
        self.__nonsuspended_mass = nonsuspended_mass

    @property
    def driver_mass(self):
        return self.__driver_mass

    @driver_mass.setter
    def driver_mass(self, driver_mass):
        self.__driver_mass = driver_mass

    @property
    def cg_height(self):
        return self.__cg_height

    @cg_height.setter
    def cg_height(self, cg_height):
        self.__cg_height = cg_height

    @property
    def weightdist_front(self):
        return self.__weightdist_front

    @weightdist_front.setter
    def weightdist_front(self, weightdist_front):
        self.__weightdist_front = weightdist_front

    '''Geometry'''
    @property
    def wheelbase(self):
        return self.__wheelbase

    @wheelbase.setter
    def wheelbase(self, wheelbase):
        self.__wheelbase = wheelbase

    @property
    def trackwidth(self):
        return self.__trackwidth

    @trackwidth.setter
    def trackwidth(self, trackwidth):
        self.__trackwidth = trackwidth

    @property
    def trackwidth_front(self):
        return self.__trackwidth_front

    @trackwidth_front.setter
    def trackwidth_front(self, trackwidth_front):
        self.__trackwidth_front = trackwidth_front

    @property
    def trackwidth_rear(self):
        return self.__trackwidth_rear

    @trackwidth_rear.setter
    def trackwidth_rear(self, trackwidth_rear):
        self.__trackwidth_rear = trackwidth_rear

    @property
    def rollcentre_front(self):
        return self.__rollcentre_front

    @rollcentre_front.setter
    def rollcentre_front(self, rollcentre_front):
        self.__rollcentre_front = rollcentre_front

    @property
    def rollcentre_rear(self):
        return self.__rollcentre_rear

    @rollcentre_rear.setter
    def rollcentre_rear(self, rollcentre_rear):
        self.__rollcentre_rear = rollcentre_rear

    @property
    def vsal_front(self):
        return self.__vsal_front

    @vsal_front.setter
    def vsal_front(self, vsal_front):
        self.__vsal_front = vsal_front

    @property
    def vsal_rear(self):
        return self.__vsal_rear

    @vsal_rear.setter
    def vsal_rear(self, vsal_rear):
        self.__vsal_rear = vsal_rear

    '''Suspension'''
    @property
    def cornerspring_front(self):
        return self.__cornerspring_front

    @cornerspring_front.setter
    def cornerspring_front(self, cornerspring_front):
        self.__cornerspring_front = cornerspring_front

    @property
    def cornerspring_rear(self):
        return self.__cornerspring_rear

    @cornerspring_rear.setter
    def cornerspring_rear(self, cornerspring_rear):
        self.__cornerspring_rear = cornerspring_rear

    @property
    def antirollstiffness_front(self):
        return self.__antirollstiffness_front

    @antirollstiffness_front.setter
    def antirollstiffness_front(self, antirollstiffness_front):
        self.__antirollstiffness_front = antirollstiffness_front

    @property
    def antirollstiffness_rear(self):
        return self.__antirollstiffness_rear

    @antirollstiffness_rear.setter
    def antirollstiffness_rear(self, antirollstiffness_rear):
        self.__antirollstiffness_rear = antirollstiffness_rear

    ''' Derived properties of the vehicle '''
    @property
    def a(self):
        return self.wheelbase * (1 - self.weightdist_front)

    @property
    def b(self):
        return self.wheelbase * self.weightdist_front

    @property
    def mass(self):
        return self.suspended_mass + self.nonsuspended_mass

    @property
    def antiroll_distribution(self):
        '''Calculates the anti-roll stiffness calculation. This is also known as the first magic number'''

        stiffness_front = self.cornerspring_front + self.antirollstiffness_front
        stiffness_rear = self.cornerspring_rear + self.antirollstiffness_rear
        return stiffness_front / (stiffness_front + stiffness_rear)

    def calc_lat_load_transfer(self, a_lat):
        '''
        Calculate the lateral load transfer using the OptimumG Seminar method.
        Several simplifying assumptions are made in the calculation.
        - Non-suspended mass CG is located at the centre of the tire
        - Excludes the tire stiffness when calculating the elastic weight transfer
        - Constant CG and roll centre locations
        '''

        # Weight transfer at the front axle
        nonsuspended_front = self.nonsuspended_mass / 2 * a_lat * self.tires.fr.re / self.trackwidth_front
        geometric_front = self.weightdist_front * self.suspended_mass * a_lat * self.rollcentre_front / self.trackwidth_front
        elastic_front = self.antiroll_distribution * self.suspended_mass * a_lat * (self.rollcentre_front - self.cg_height) / self.trackwidth_front

        # Weight transfer at the rear axle
        nonsuspended_rear = self.nonsuspended_mass / 2 * a_lat * self.tires.rr.re / self.trackwidth_rear
        geometric_rear = (1 - self.weightdist_front) * self.suspended_mass * a_lat * self.rollcentre_rear / self.trackwidth_rear
        elastic_rear = (1 - self.antiroll_distribution) * self.suspended_mass * a_lat * (self.rollcentre_rear - self.cg_height) / self.trackwidth_rear

        # Package the components nicely in a dictionary
        loadtransfer = {'nonsuspended_front': nonsuspended_front,
                        'geometric_front': geometric_front,
                        'elastic_front': elastic_front,
                        'nonsuspended_rear': nonsuspended_rear,
                        'geometric_rear': geometric_rear,
                        'elastic_rear': elastic_rear
                        }

        # Return all components of the weight transfer. Think of a better name for this.
        return loadtransfer

    def calc_vertical_load(self, a_lat, a_long, mode=None):
        '''Calculate the tire vertical load'''

        loadtransfer = self.calc_lat_load_transfer(a_lat)

        # Sum the components of the weight transfer
        front_lat_trnsfr = loadtransfer['nonsuspended_front'] + loadtransfer['geometric_front'] + loadtransfer['elastic_front']
        rear_lat_trnsfr = loadtransfer['nonsuspended_rear'] + loadtransfer['geometric_rear'] + loadtransfer['elastic_rear']

        # Wheel vertical load without weight transfer effects for a single whel
        fz_front_wheel = -(9.81 * self.mass * self.weightdist_front / 2)
        fz_rear_wheel = -(9.81 * self.mass * (1 - self.weightdist_front) / 2)

        # Wheel lift assumption - for a given axle, wheel will take entire axle load
        if math.fabs(front_lat_trnsfr) > math.fabs(fz_front_wheel):
            front_lat_trnsfr = math.copysign(fz_front_wheel, front_lat_trnsfr)

        if math.fabs(rear_lat_trnsfr) > math.fabs(fz_rear_wheel):
            rear_lat_trnsfr = math.copysign(fz_rear_wheel, rear_lat_trnsfr)

        fz_fr = fz_front_wheel + front_lat_trnsfr
        fz_fl = fz_front_wheel - front_lat_trnsfr
        fz_rr = fz_rear_wheel + rear_lat_trnsfr
        fz_rl = fz_rear_wheel - rear_lat_trnsfr

        return Quartet(fz_fr, fz_fl, fz_rr, fz_rl)

    def calc_roll_moment(self, theta):
        '''Calculate the antiroll stiffness for a given roll angle in radians'''

        wheelrate_front = self.cornerspring_front + self.antirollstiffness_front
        wheelrate_rear = self.cornerspring_rear + self.antirollstiffness_rear

        antiroll_front = self.trackwidth_front ** 2 * math.tan(theta) * wheelrate_front / 2
        antiroll_rear = self.trackwidth_rear ** 2 * math.tan(theta) * wheelrate_rear / 2

        return antiroll_front + antiroll_rear

    def calc_roll_angle(self, a_lat):
        '''Calculate the chassis roll angle when subjected to a lateral acceleration'''

        loadtransfer = self.calc_lat_load_transfer(a_lat)

        # Calculate the total roll moment to be reacted
        roll_moment = loadtransfer['elastic_front'] * self.trackwidth_front + loadtransfer['elastic_rear'] * self.trackwidth_rear
        roll_equation = lambda theta: self.calc_roll_moment(theta) - roll_moment
        roll_angle = fsolve(roll_equation, 0)

        return roll_angle.flat[0]

    def calc_camber_angles(self, a_lat):
        '''Calculate the tire camber variation using constant VSAL approximation'''

        # Determine the chassis roll angle
        roll_angle = self.calc_roll_angle(a_lat)

        # With the roll angle known, determine the vertical displacment required to return the wheel
        displacement_front = self.trackwidth_front / 2 * math.tan(roll_angle)
        displacement_rear = self.trackwidth_rear / 2 * math.tan(roll_angle)

        # Inclination angle in the 'vehicle' frame of reference
        gamma_fr = -roll_angle + math.atan(displacement_front / self.vsal_front)
        gamma_fl = roll_angle - math.atan(displacement_front / self.vsal_front)
        gamma_rr = -roll_angle + math.atan(displacement_rear / self.vsal_rear)
        gamma_rl = roll_angle - math.atan(displacement_rear / self.vsal_rear)

        return Quartet(gamma_fr, gamma_fl, gamma_rr, gamma_rl)

    def calc_slip_angles(self, velocity, yaw_speed, beta):
        '''Calculate the tire slip angles'''

        velocity_y = velocity*math.tan(beta)

        alpha_fr = math.atan((velocity_y + self.a*yaw_speed)/(velocity - self.trackwidth/2 * yaw_speed))
        alpha_fl = math.atan((velocity_y + self.a*yaw_speed)/(velocity + self.trackwidth/2 * yaw_speed))
        alpha_rr = math.atan((velocity_y - self.b*yaw_speed)/(velocity - self.trackwidth/2 * yaw_speed))
        alpha_rl = math.atan((velocity_y - self.b*yaw_speed)/(velocity + self.trackwidth/2 * yaw_speed))

        return Quartet(alpha_fr, alpha_fl, alpha_rr, alpha_rl)

    def calc_lateral_forces(self, fz, alpha, delta, gamma):
        '''Calculate the lateral force generated by the tires'''

        # Ignore camber effects for now
        fy_fr = self.tires.fr.calc_fy(fz.fr, alpha.fr+delta, 0, -gamma.fr)
        fy_fl = -self.tires.fl.calc_fy(fz.fl, -(alpha.fl+delta), 0, -gamma.fl)
        fy_rr = self.tires.rr.calc_fy(fz.rr, alpha.rr, 0, -gamma.rr)
        fy_rl = -self.tires.rl.calc_fy(fz.rl, -alpha.rl, 0, -gamma.rl)

        return Quartet(fy_fr, fy_fl, fy_rr, fy_rl)

    def calc_self_aligning(self, fz, alpha, delta, gamma):
        '''Calculate the self aligning torque generated by the tires'''

        mz_fr = self.tires.fr.calc_mz(fz.fr, alpha.fr+delta, 0, -gamma.fr)
        mz_fl = -self.tires.fl.calc_mz(fz.fl, -(alpha.fl+delta), 0, -gamma.fl)
        mz_rr = self.tires.rr.calc_mz(fz.rr, alpha.rr, 0, -gamma.rr)
        mz_rl = -self.tires.rl.calc_mz(fz.rl, -alpha.rl, 0, -gamma.rl)

        return Quartet(mz_fr, mz_fl, mz_rr, mz_rl)

    def calc_vehicle_forces(self, velocity, yaw_speed, a_lat, beta, delta):
        '''Calculate the resolved forces and moments acting on the car'''

        alpha = self.calc_slip_angles(velocity, yaw_speed, beta)
        gamma = self.calc_camber_angles(a_lat)

        fz = self.calc_vertical_load(a_lat, 0)  # assume a_long is zero
        fy = self.calc_lateral_forces(fz, alpha, delta, gamma)
        mz = self.calc_self_aligning(fz, alpha, delta, gamma)

        # Transformation matrix
        a11 = math.sin(delta)
        a12 = math.sin(delta)
        a13 = 0
        a14 = 0
        a21 = math.cos(delta)
        a22 = math.cos(delta)
        a23 = 1
        a24 = 1
        a31 = (self.trackwidth/2*math.sin(delta) + self.a*math.cos(delta))
        a32 = (-self.trackwidth/2*math.sin(delta) + self.a*math.cos(delta))
        a33 = -self.b
        a34 = -self.b

        matrix = np.matrix([[a11, a12, a13, a14],   # F_x
                            [a21, a22, a23, a24],   # F_y
                            [a31, a32, a33, a34]])  # M_z

        # FR, FL, RR, RL
        forces = np.matrix([[fy.fr], [fy.fl], [fy.rr], [fy.rl]])

        # Rotate forces and moments to the vehicle frame of reference
        resolved = matrix * forces

        # Add in the self-aligning torques
        resolved.flat[2] = resolved.flat[2] + mz.fr + mz.fl + mz.rr + mz.rl

        return resolved
