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

        # Derived, Mass and Dimensions
        self.a = self.wheelbase * (1 - self.weightdist_front)
        self.b = self.wheelbase * self.weightdist_front

        # Vehicle State
        # Velocities
        self.velocity = 0
        self.yaw_rate = 0
        self.a_lat = 0
        self.a_long = 0

        # Steering
        self.delta = 0

        # Vertical Tire Load
        self.fz_fr = 0
        self.fz_fl = 0
        self.fz_rr = 0
        self.fz_rl = 0
        self.calc_tireload()

        # Tire slip angle
        self.alpha_fr = 0
        self.alpha_fl = 0
        self.alpha_rr = 0
        self.alpha_rl = 0

    def calc_latloadtrnsfr(self, a_lat=None):
        if a_lat is None:
            a_lat = self.a_lat

        # Highly simplified model - neglect kinematics and suspension
        return (self.mass * self.cg_height * a_lat) / self.trackwidth

    def calc_longloadtrnsfr(self, a_long=None):
        ''' UNUSED FOR NOW'''
        if a_long is None:
            a_long = self.a_long

        return (self.mass * self.cg_height * a_long) / self.wheelbase

    def calc_tireload(self, a_lat=None, a_long=None):
        if a_lat is None:
            a_lat = self.a_lat

        if a_long is None:
            a_long = self.a_long

        # Calculate load transfer
        lat_trnsfr = self.calc_latloadtrnsfr()
        front_lat_trnsfr = lat_trnsfr * self.latloadtrnsfr_dist
        rear_lat_trnsfr = lat_trnsfr * (1 - self.latloadtrnsfr_dist)

        self.fz_fr = -(9.81 * self.mass * self.weightdist_front / 2) + front_lat_trnsfr
        self.fz_fl = -(9.81 * self.mass * self.weightdist_front / 2) - front_lat_trnsfr
        self.fz_rr = -(9.81 * self.mass * (1 - self.weightdist_front) / 2) + rear_lat_trnsfr
        self.fz_rl = -(9.81 * self.mass * (1 - self.weightdist_front) / 2) - rear_lat_trnsfr

        return self.fz_fr, self.fz_fl, self.fz_rr, self.fz_rl

    def update_tireslip(self):
        velocity_y = self.velocity*math.tan(self.beta)

        self.alpha_fr = math.atan((velocity_y +
            self.a*self.yaw_rate)/(self.velocity - self.trackwidth/2 * self.yaw_rate))
        self.alpha_fl = math.atan((velocity_y +
            self.a*self.yaw_rate)/(self.velocity + self.trackwidth/2 * self.yaw_rate))
        self.alpha_rr = math.atan((velocity_y -
            self.b*self.yaw_rate)/(self.velocity - self.trackwidth/2 * self.yaw_rate))
        self.alpha_rl = math.atan((velocity_y -
            self.b*self.yaw_rate)/(self.velocity + self.trackwidth/2 * self.yaw_rate))

        return self.alpha_fr, self.alpha_fl, self.alpha_rr, self.alpha_rl

    def calc_tire_latforce(self, delta=None):
        if delta is None:
            delta = self.delta

        # Ignore camber effects for now
        fy_fr = self.tire_fr.calc_fy(self.fz_fr, self.alpha_fr+delta, 0, 0)
        fy_fl = -self.tire_fl.calc_fy(self.fz_fl, -(self.alpha_fl+delta), 0, 0)
        fy_rr = self.tire_rr.calc_fy(self.fz_rr, self.alpha_rr, 0, 0)
        fy_rl = -self.tire_rl.calc_fy(self.fz_rl, -self.alpha_rl, 0, 0)

        return fy_fr, fy_fl, fy_rr, fy_rl

    def calc_tire_selfalign(self, delta=None):
        if delta is None:
            delta = self.delta

        mz_fr = self.tire_fr.calc_mz(self.fz_fr, self.alpha_fr+delta, 0, 0)
        mz_fl = -self.tire_fl.calc_mz(self.fz_fl, -(self.alpha_fl+delta), 0, 0)
        mz_rr = self.tire_rr.calc_mz(self.fz_rr, self.alpha_rr, 0, 0)
        mz_rl = -self.tire_rl.calc_mz(self.fz_rl, -self.alpha_rl, 0, 0)

        return mz_fr, mz_fl, mz_rr, mz_rl

    def resolve_forces(self, delta=None):
        if delta is None:
            delta = self.delta

        fy_fr, fy_fl, fy_rr, fy_rl = self.calc_tire_latforce(delta)
        mz_fr, mz_fl, mz_rr, mz_rl = self.calc_tire_selfalign(delta)

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
