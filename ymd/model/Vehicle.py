import math
import numpy as np


class Vehicle:
    def __init__(self, tire_fr, tire_fl, tire_rr, tire_rl, dim):
        # Tires
        self.tire_fr = tire_fr
        self.tire_fl = tire_fl
        self.tire_rr = tire_rr
        self.tire_rl = tire_rl

        # Mass and Dimensions
        self.cg_height = dim['cg_height']  # CG height [m]
        self.weightdist_front = dim['weightdist_front']  # front weight dist [%]
        self.trackwidth = dim['trackwidth']  # track wdith [m]
        self.wheelbase = dim['wheelbase']  # wheelbase [m]
        self.latloadtrnsfr_dist = dim['latloadtrnsfr']  # lateral load transfer, distribution, front [%]
        self.mass = dim['mass']

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

    @property
    def trackwidth(self):
        return self.__trackwidth

    @trackwidth.setter
    def trackwidth(self, trackwidth):
        self.__trackwidth = trackwidth

    @property
    def wheelbase(self):
        return self.__wheelbase

    @wheelbase.setter
    def wheelbase(self, wheelbase):
        self.__wheelbase = wheelbase

    @property
    def latloadtrnsfr_dist(self):
        return self.__latloadtrnsfr_dist

    @latloadtrnsfr_dist.setter
    def latloadtrnsfr_dist(self, latloadtrnsfr_dist):
        self.__latloadtrnsfr_dist = latloadtrnsfr_dist

    @property
    def mass(self):
        return self.__mass

    @mass.setter
    def mass(self, mass):
        self.__mass = mass

    ''' Derived properties of the vehicle '''
    @property
    def a(self):
        return self.wheelbase * (1 - self.weightdist_front)

    @property
    def b(self):
        return self.wheelbase * self.weightdist_front

    def calc_vertical_load(self, a_lat, a_long):
        '''Calculate the tire vertical load with simplified weight transfer '''

        lat_trnsfr = (self.mass * self.cg_height * a_lat) / self.trackwidth
        long_trnsfr = (self.mass * self.cg_height * a_long) / self.wheelbase

        front_lat_trnsfr = lat_trnsfr * self.latloadtrnsfr_dist
        rear_lat_trnsfr = lat_trnsfr * (1 - self.latloadtrnsfr_dist)

        fz_fr = -(9.81 * self.mass * self.weightdist_front / 2) + front_lat_trnsfr
        fz_fl = -(9.81 * self.mass * self.weightdist_front / 2) - front_lat_trnsfr
        fz_rr = -(9.81 * self.mass * (1 - self.weightdist_front) / 2) + rear_lat_trnsfr
        fz_rl = -(9.81 * self.mass * (1 - self.weightdist_front) / 2) - rear_lat_trnsfr

        return fz_fr, fz_fl, fz_rr, fz_rl

    def calc_slip_angles(self, velocity, yaw_rate, beta):
        '''Calculate the tire slip angles'''

        velocity_y = velocity*math.tan(beta)

        alpha_fr = math.atan((velocity_y + self.a*yaw_rate)/(velocity - self.trackwidth/2 * yaw_rate))
        alpha_fl = math.atan((velocity_y + self.a*yaw_rate)/(velocity + self.trackwidth/2 * yaw_rate))
        alpha_rr = math.atan((velocity_y - self.b*yaw_rate)/(velocity - self.trackwidth/2 * yaw_rate))
        alpha_rl = math.atan((velocity_y - self.b*yaw_rate)/(velocity + self.trackwidth/2 * yaw_rate))

        return alpha_fr, alpha_fl, alpha_rr, alpha_rl

    def calc_lateral_forces(self, fz_fr, fz_fl, fz_rr, fz_rl, alpha_fr, alpha_fl, alpha_rr, alpha_rl, delta):
        '''Calculate the lateral force generated by the tires'''

        # Ignore camber effects for now
        fy_fr = self.tire_fr.calc_fy(fz_fr, alpha_fr+delta, 0, 0)
        fy_fl = -self.tire_fl.calc_fy(fz_fl, -(alpha_fl+delta), 0, 0)
        fy_rr = self.tire_rr.calc_fy(fz_rr, alpha_rr, 0, 0)
        fy_rl = -self.tire_rl.calc_fy(fz_rl, -alpha_rl, 0, 0)

        return fy_fr, fy_fl, fy_rr, fy_rl

    def calc_self_aligning(self, fz_fr, fz_fl, fz_rr, fz_rl, alpha_fr, alpha_fl, alpha_rr, alpha_rl, delta):
        '''Calculate the self aligning torque generated by the tires'''

        mz_fr = self.tire_fr.calc_mz(fz_fr, alpha_fr+delta, 0, 0)
        mz_fl = -self.tire_fl.calc_mz(fz_fl, -(alpha_fl+delta), 0, 0)
        mz_rr = self.tire_rr.calc_mz(fz_rr, alpha_rr, 0, 0)
        mz_rl = -self.tire_rl.calc_mz(fz_rl, -alpha_rl, 0, 0)

        return mz_fr, mz_fl, mz_rr, mz_rl

    def calc_vehicle_forces(self, velocity, yaw_rate, a_lat, beta, delta):
        '''Calculate the resolved forces and moments acting on the car'''

        alpha_fr, alpha_fl, alpha_rr, alpha_rl = self.calc_slip_angles(velocity, yaw_rate, beta)
        fz_fr, fz_fl, fz_rr, fz_rl = self.calc_vertical_load(a_lat, 0)  # assume a_long is zero
        fy_fr, fy_fl, fy_rr, fy_rl = self.calc_lateral_forces(fz_fr, fz_fl, fz_rr, fz_rl, alpha_fr, alpha_fl, alpha_rr, alpha_rl, delta)
        mz_fr, mz_fl, mz_rr, mz_rl = self.calc_self_aligning(fz_fr, fz_rl, fz_rr, fz_rl, alpha_fr, alpha_fl, alpha_rr, alpha_rl, delta)

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
        forces = np.matrix([[fy_fr], [fy_fl], [fy_rr], [fy_rl]])

        # TODO: Find a better name for this
        resolved = matrix * forces
        resolved.flat[2] = resolved.flat[2] + mz_fr + mz_fl + mz_rr + mz_rl

        return resolved
