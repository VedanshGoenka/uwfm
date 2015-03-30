import configparser
import math
import matplotlib.pyplot as plt
import numpy as np

from model.pacejka_MF52 import PacejkaMF52 as pacejka


def solver(trackwidth, a, b, delta, fy_fr, fy_fl, fy_rr, fy_rl):
    # Transformation matrix
    a11 = math.sin(delta)
    a12 = math.sin(delta)
    a13 = 0
    a14 = 0
    a21 = math.cos(delta)
    a22 = math.cos(delta)
    a23 = 1
    a24 = 1
    a31 = (trackwidth/2*math.sin(delta) + a*math.cos(delta))
    a32 = (-trackwidth/2*math.sin(delta) + a*math.cos(delta))
    a33 = -b
    a34 = -b

    matrix = np.matrix([[a11, a12, a13, a14],   # F_x
                        [a21, a22, a23, a24],   # F_y
                        [a31, a32, a33, a34]])  # M_z

    # FR, FL, RR, RL
    forces = np.matrix([[fy_fr], [fy_fl], [fy_rr], [fy_rl]])

    return matrix * forces


def load_transfer(mass, accel, trackwidth, cg_height):
    # Highly simplified model - neglect kinematics and suspension
    return (9.81 * mass * cg_height * accel) / trackwidth


def tire_load(mass, accel, trackwidth, cg_height, weightdist, aloadtrnsfrdist):
    delta_load = load_transfer(mass, accel, trackwidth, cg_height)
    f_trnsfr = delta_load * aloadtrnsfrdist
    r_trnsfr = delta_load * (1 - aloadtrnsfrdist)

    fz_fr = -(9.81 * mass * weightdist / 2) + f_trnsfr
    fz_fl = -(9.81 * mass * weightdist / 2) - f_trnsfr
    fz_rr = -(9.81 * mass * (1 - weightdist) / 2) + r_trnsfr
    fz_rl = -(9.81 * mass * (1 - weightdist) / 2) - r_trnsfr

    return fz_fr, fz_fl, fz_rr, fz_rl


def main():
    # define vehicle parameters
    cgheight = 0/1000    # in metres [m]
    wheelbase = 1650/1000  # in metres [m]
    trackwidth = 1250/1000  # in metres [m]
    weightdist = .50  # percentage, front [%]
    mass = 250  # kilograms [kg]
    aloadtrnsfrdist = .50  # percentage, front [%]
    velocity = 30 # in metres per second [m/s]

    # define test conditions
    error = 0.0001
    beta_sweep = np.arange(-15, 15)
    beta_sweep = [math.radians(value) for value in beta_sweep]
    delta_sweep = np.arange(-15, 15)
    delta_sweep = [math.radians(value) for value in delta_sweep]

    # read in tire data
    config = configparser.ConfigParser()
    config.read('data/tire/hoosier_lc0_sorted.ini')

    # Extract coefficients
    general = {key: float(config['general'][key]) for key in config['general']}
    py = {key: float(config['lateral'][key]) for key in config['lateral']}
    qz = {key: float(config['self_aligning'][key]) for key in config['self_aligning']}
    px = {key: float(config['longitudinal'][key]) for key in config['longitudinal']}
    qx = {key: float(config['overturning'][key]) for key in config['overturning']}
    rx = {key: float(config['longitudinal_combined'][key]) for key in config['longitudinal_combined']}
    ry = {key: float(config['lateral_combined'][key]) for key in config['lateral_combined']}
    sz = {key: float(config['self_aligning_combined'][key]) for key in config['self_aligning_combined']}
    qy = {key: float(config['roll_moment'][key]) for key in config['roll_moment']}

    # build tire model
    tire_model = pacejka(general, py, qz, px, qx, rx, ry, sz, qy)

    # calculate vehicle characteristics
    a = wheelbase * (1 - weightdist)
    b = wheelbase - a

    # static vertical load
    staticfz_fr, staticfz_fl, staticfz_rr, staticfz_rl = tire_load(mass, 0, trackwidth, cgheight,
                                                                   weightdist, aloadtrnsfrdist)

    result_ay = np.empty([len(beta_sweep), len(delta_sweep)])
    result_mz = np.empty([len(beta_sweep), len(delta_sweep)])

    for i, beta in enumerate(beta_sweep):
        for j, delta in enumerate(delta_sweep):
            # Initialize vertical load variables
            fz_fr = staticfz_fr
            fz_fl = staticfz_fl
            fz_rr = staticfz_rr
            fz_rl = staticfz_rl

            m_z = 0
            a_lat = 0
            a_lat_prev = 100

            while math.fabs(a_lat - a_lat_prev) > error:
                # TODO: cannot guarantee that the solver runs
                # TODO: check coordinate system!
                # TODO: rethink variable naming

                # Initialize previous value
                a_lat_prev = a_lat

                # Calculate yaw velocity
                yaw_rate = a_lat/velocity

                # Calculate slip angles 
                velocity_y = velocity*math.tan(beta)
                alpha_fr = math.atan((velocity_y + a*yaw_rate)/(velocity - trackwidth/2 * yaw_rate))
                alpha_fl = math.atan((velocity_y + a*yaw_rate)/(velocity + trackwidth/2 * yaw_rate))
                alpha_rr = math.atan((velocity_y - b*yaw_rate)/(velocity - trackwidth/2 * yaw_rate))
                alpha_rl = math.atan((velocity_y - b*yaw_rate)/(velocity + trackwidth/2 * yaw_rate))

                # Calculate tire forces
                fy_fr = tire_model.calc_fy(fz_fr, alpha_fr+delta, 0, 0)
                fy_fl = -tire_model.calc_fy(fz_fl, -(alpha_fl+delta), 0, 0)
                fy_rr = tire_model.calc_fy(fz_rr, alpha_rr, 0, 0)
                fy_rl = -tire_model.calc_fy(fz_rl, -alpha_rl, 0, 0)

                print(beta, alpha_fr, alpha_fl, alpha_rr, alpha_rl)
                #print(a_lat, fy_fr, fy_fl, fy_rr, fy_rl)
                #print(fz_fr, fz_rl, fz_rr, fz_rl)

                # Solve for resulting forces and moments
                solution = solver(trackwidth, a, b, delta, fy_fr, fy_fl, fy_rr, fy_rl)

                a_lat = solution.flat[1]/mass
                m_z = solution.flat[2]

                fz_fr, fz_fl, fz_rr, fz_rl = tire_load(mass, a_lat/9.81, trackwidth, cgheight,
                                                       weightdist, aloadtrnsfrdist)

            mz_fr = tire_model.calc_mz(fz_fr, beta+delta, 0, 0)
            mz_fl = -tire_model.calc_mz(fz_fl, -(beta+delta), 0, 0)
            mz_rr = tire_model.calc_mz(fz_rr, beta, 0, 0)
            mz_rl = -tire_model.calc_mz(fz_rl, -beta, 0, 0)

            result_ay[i][j] = a_lat / 9.81
            result_mz[i][j] = m_z + mz_fr + mz_fl + mz_rr + mz_rl

    plt.plot(result_ay[:][:], result_mz[:][:])
    plt.plot(np.transpose(result_ay)[:][:],
             np.transpose(result_mz)[:][:])
    plt.xlabel('Lateral Acceleration [g]')
    plt.ylabel('Yaw Moment [Nm]')
    plt.title('Yaw Moment Diagram')
    plt.grid(True)
    plt.show()
    plt.savefig('ymd.png')


if __name__ == "__main__":
    main()
