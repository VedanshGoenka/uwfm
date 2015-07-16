import math
import numpy as np


class Quartet:
    '''An arbitrary class that holds the four corner values of a car.'''
    def __init__(self, fr, fl, rr, rl):
        self.quartet = [fr, fl, rr, rl]

    @property
    def quartet(self):
        return self.__quartet

    @quartet.setter
    def quartet(self, data):
        # This is awful naming
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
    def __init__(self, tire_fr, tire_fl, tire_rr, tire_rl, mass, geometry, suspension):
        # TODO: The parameter naming sucks

        # Tires
        self.tire_fr = tire_fr
        self.tire_fl = tire_fl
        self.tire_rr = tire_rr
        self.tire_rl = tire_rl

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
# Suspension self.latloadtrnsfr_dist = suspension['latloadtrnsfr']  # lateral load transfer, distribution, front [%]
        self.arb_stiffness_ratio = suspension['arb_stiffness_ratio']

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

    '''Suspension'''
    @property
    def arb_stiffness_ratio(self):
        return self.__arb_stiffness_ratio

    @arb_stiffness_ratio.setter
    def arb_stiffness_ratio(self, arb_stiffness_ratio):
        self.__arb_stiffness_ratio = arb_stiffness_ratio

    @property
    def roll_stiffness(self):
        return self.__roll_stiffness

    @roll_stiffness.setter
    def roll_stiffness(self, roll_stiffness):
        self.__roll_stiffness = roll_stiffness

    @property
    def latloadtrnsfr_dist(self):
        return self.__latloadtrnsfr_dist

    @latloadtrnsfr_dist.setter
    def latloadtrnsfr_dist(self, latloadtrnsfr_dist):
        self.__latloadtrnsfr_dist = latloadtrnsfr_dist

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

    def calc_vertical_load(self, a_lat, a_long, mode=None):
        '''Calculate the tire vertical load with simplified weight transfer '''

        # Wheel vertical load without weight transfer effects for a single whel
        fz_front_wheel = -(9.81 * self.mass * self.weightdist_front / 2)
        fz_rear_wheel = -(9.81 * self.mass * (1 - self.weightdist_front) / 2)

        if mode is 'simple':
            lat_trnsfr = (self.mass * self.cg_height * a_lat) / self.trackwidth
            # Unused for now
            # long_trnsfr = (self.mass * self.cg_height * a_long) / self.wheelbase

            front_lat_trnsfr = lat_trnsfr * self.latloadtrnsfr_dist
            rear_lat_trnsfr = lat_trnsfr * (1 - self.latloadtrnsfr_dist)
        else:
            # Calculate the load transfer on the front axle. WARNING: lots of assumptions being made here!
            nonsuspended_front = self.nonsuspended_mass / 2 * a_lat * self.tire_fr.re / self.trackwidth_front
            geometric_front = (self.weightdist_front * self.suspended_mass *
                               a_lat * self.rollcentre_front / self.trackwidth_front)
            elastic_front = (self.arb_stiffness_ratio * self.weightdist_front * self.suspended_mass *
                             a_lat * (self.rollcentre_front - self.cg_height) / self.trackwidth_front)
            front_lat_trnsfr = nonsuspended_front + geometric_front + elastic_front

            nonsuspended_rear = self.nonsuspended_mass / 2 * a_lat * self.tire_rr.re / self.trackwidth_rear
            geometric_rear = ((1 - self.weightdist_front) * self.suspended_mass *
                              a_lat * self.rollcentre_rear / self.trackwidth_rear)
            elastic_rear = ((1 - self.arb_stiffness_ratio) * (1 - self.weightdist_front) * self.suspended_mass *
                            a_lat * (self.rollcentre_rear - self.cg_height) / self.trackwidth_rear)
            rear_lat_trnsfr = nonsuspended_rear + geometric_rear + elastic_rear

        # Wheel lift assumption - for a given axle, wheel will take entire axle load
        if math.fabs(front_lat_trnsfr) > math.fabs(fz_front_wheel):
            front_lat_trnsfr = math.copysign(fz_front_wheel, front_lat_trnsfr)
            print('Front Wheel Lift!')

        if math.fabs(rear_lat_trnsfr) > math.fabs(fz_rear_wheel):
            rear_lat_trnsfr = math.copysign(fz_rear_wheel, rear_lat_trnsfr)
            print('Rear Wheel Lift!')

        fz_fr = fz_front_wheel + front_lat_trnsfr
        fz_fl = fz_front_wheel - front_lat_trnsfr
        fz_rr = fz_rear_wheel + rear_lat_trnsfr
        fz_rl = fz_rear_wheel - rear_lat_trnsfr

        return Quartet(fz_fr, fz_fl, fz_rr, fz_rl)

    def calc_slip_angles(self, velocity, yaw_speed, beta):
        '''Calculate the tire slip angles'''

        velocity_y = velocity*math.tan(beta)

        alpha_fr = math.atan((velocity_y + self.a*yaw_speed)/(velocity - self.trackwidth/2 * yaw_speed))
        alpha_fl = math.atan((velocity_y + self.a*yaw_speed)/(velocity + self.trackwidth/2 * yaw_speed))
        alpha_rr = math.atan((velocity_y - self.b*yaw_speed)/(velocity - self.trackwidth/2 * yaw_speed))
        alpha_rl = math.atan((velocity_y - self.b*yaw_speed)/(velocity + self.trackwidth/2 * yaw_speed))

        return Quartet(alpha_fr, alpha_fl, alpha_rr, alpha_rl)

    def calc_lateral_forces(self, fz, alpha, delta):
        '''Calculate the lateral force generated by the tires'''

        # Ignore camber effects for now
        fy_fr = self.tire_fr.calc_fy(fz.fr, alpha.fr+delta, 0, 0)
        fy_fl = -self.tire_fl.calc_fy(fz.fl, -(alpha.fl+delta), 0, 0)
        fy_rr = self.tire_rr.calc_fy(fz.rr, alpha.rr, 0, 0)
        fy_rl = -self.tire_rl.calc_fy(fz.rl, -alpha.rl, 0, 0)

        return Quartet(fy_fr, fy_fl, fy_rr, fy_rl)

    def calc_self_aligning(self, fz, alpha, delta):
        '''Calculate the self aligning torque generated by the tires'''

        mz_fr = self.tire_fr.calc_mz(fz.fr, alpha.fr+delta, 0, 0)
        mz_fl = -self.tire_fl.calc_mz(fz.fl, -(alpha.fl+delta), 0, 0)
        mz_rr = self.tire_rr.calc_mz(fz.rr, alpha.rr, 0, 0)
        mz_rl = -self.tire_rl.calc_mz(fz.rl, -alpha.rl, 0, 0)

        return Quartet(mz_fr, mz_fl, mz_rr, mz_rl)

    def calc_vehicle_forces(self, velocity, yaw_speed, a_lat, beta, delta):
        '''Calculate the resolved forces and moments acting on the car'''

        alpha = self.calc_slip_angles(velocity, yaw_speed, beta)
        fz = self.calc_vertical_load(a_lat, 0)  # assume a_long is zero
        fy = self.calc_lateral_forces(fz, alpha, delta)
        mz = self.calc_self_aligning(fz, alpha, delta)

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

        # TODO: Find a better name for this
        resolved = matrix * forces
        resolved.flat[2] = resolved.flat[2] + mz.fr + mz.fl + mz.rr + mz.rl

        return resolved
