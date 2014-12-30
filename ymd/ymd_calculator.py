import configparser
import math
import matplotlib.pyplot as plt
import numpy as np

from model.pacejka94 import Pacejka94 as pacejka


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

    fz_fr = (9.81 * mass * weightdist / 2) - f_trnsfr
    fz_fl = (9.81 * mass * weightdist / 2) + f_trnsfr
    fz_rr = (9.81 * mass * (1 - weightdist) / 2) - r_trnsfr
    fz_rl = (9.81 * mass * (1 - weightdist) / 2) + r_trnsfr

    return fz_fr, fz_fl, fz_rr, fz_rl


def main():
    # define vehicle parameters
    cgheight = 300/1000    # in metres [m]
    wheelbase = 1650/1000  # in metres [m]
    trackwidth = 1250/1000  # in metres [m]
    weightdist = .40  # percentage, front [%]
    mass = 250  # kilograms [kg]
    aloadtrnsfrdist = .40  # percentage, front [%]

    # define test conditions
    error = 0.0001
    beta_sweep = np.arange(-8, 9)
    beta_sweep = [math.radians(value) for value in beta_sweep]
    delta_sweep = np.arange(-9, 10)
    delta_sweep = [math.radians(value) for value in delta_sweep]

    # read in tire data
    config = configparser.ConfigParser()
    config.read('data/tire/generic.ini')
    coeff = {key: float(config['lateral'][key]) for key in config['lateral']}

    # build tire model
    tire_model = pacejka(coeff)

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

                # Initialize previous value
                a_lat_prev = a_lat

                # Calculate tire forces
                fy_fr = -tire_model.fy(fz_fr, -(beta-delta), 0)
                fy_fl = tire_model.fy(fz_fl, beta-delta, 0)
                fy_rr = -tire_model.fy(fz_rr, -beta, 0)
                fy_rl = tire_model.fy(fz_rl, beta, 0)

                # Solve for resulting forces and moments
                solution = solver(trackwidth, a, b, delta, fy_fr, fy_fl, fy_rr, fy_rl)

                a_lat = solution.flat[1]/mass
                m_z = solution.flat[2]

                fz_fr, fz_fl, fz_rr, fz_rl = tire_load(mass, a_lat/9.81, trackwidth, cgheight,
                                                       weightdist, aloadtrnsfrdist)

            result_ay[i][j] = a_lat / 9.81
            result_mz[i][j] = m_z

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
