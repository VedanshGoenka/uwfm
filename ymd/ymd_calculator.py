import configparser
import math
import matplotlib.pyplot as plt
import numpy as np

from model.pacejka_MF52 import PacejkaMF52 as Pacejka
from model.Vehicle import Vehicle

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
    # define test conditions
    velocity = 20  # [m/s]
    error = 0.0001
    beta_sweep = np.arange(-15, 15)
    beta_sweep = [math.radians(value) for value in beta_sweep]
    delta_sweep = np.arange(-15, 15)
    delta_sweep = [math.radians(value) for value in delta_sweep]

    # Load tire data and build model
    config = configparser.ConfigParser()
    config.read('data/tire/hoosier_lc0_sorted.ini')

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
    tire_model = Pacejka(general, py, qz, px, qx, rx, ry, sz, qy)

    # Load vehicle data and build model
    config.read('data/vehicle/generic_fsae.ini')
    dimensions = {key: float(config['dimensions'][key]) for key in config['dimensions']}

    car = Vehicle(tire_model, tire_model, tire_model, tire_model, dimensions)
    car.velocity = velocity

    # Initialize results arraw
    result_ay = np.empty([len(beta_sweep), len(delta_sweep)])
    result_mz = np.empty([len(beta_sweep), len(delta_sweep)])

    for i, beta in enumerate(beta_sweep):
        # Set vehicle slip angle
        car.beta = beta

        for j, delta in enumerate(delta_sweep):
            # Set static tire load
            car.a_lat = 0
            car.a_long = 0

            car.delta = delta

            # Initialize variables
            m_z = 0
            a_lat = 0
            a_lat_prev = 100

            while math.fabs(a_lat - a_lat_prev) > error:
                # Initialize previous value
                a_lat_prev = a_lat

                # Set vehicle properties
                car.a_lat = a_lat
                car.yaw_rate = car.a_lat/car.velocity
                car.calc_tireload()

                # Update tire slip angles
                car.update_tireslip()

                # Solve the vehicle
                solution = car.resolve_forces()

                a_lat = solution.flat[1]/car.mass
                m_z = solution.flat[2]

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
