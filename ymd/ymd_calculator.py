import configparser
import math
import matplotlib.pyplot as plt
import numpy as np

from model.Pacejka_MF52 import PacejkaMF52 as Pacejka
from model.Vehicle import Vehicle


def main():
    # define test conditions
    velocity = 70/3.6  # [m/s]
    error = 0.0001
    beta_sweep = np.arange(-12, 12)
    beta_sweep = [math.radians(value) for value in beta_sweep]
    delta_sweep = np.arange(-12, 12)
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
            relaxation = 0.5 

            while math.fabs(a_lat - a_lat_prev) > error:
                # Set vehicle properties
                a_lat = a_lat * relaxation + a_lat_prev * (1 - relaxation)
                yaw_rate = a_lat/velocity

                # Initialize previous value
                a_lat_prev = a_lat

                # Solve the vehicle
                solution = car.calc_vehicle_forces(velocity, yaw_rate, a_lat, beta, delta)

                a_lat = solution.flat[1]/car.mass
                m_z = solution.flat[2]

            result_ay[i][j] = a_lat / 9.81
            result_mz[i][j] = m_z

    plt.plot(np.transpose(result_ay)[:][:],
             np.transpose(result_mz)[:][:], color='black')
    plt.plot(result_ay[:][:], result_mz[:][:], color='red')
    plt.xlabel('Lateral Acceleration [g]')
    plt.ylabel('Yaw Moment [Nm]')
    plt.title('Yaw Moment Diagram')
    plt.grid(True)
    plt.show()
    plt.savefig('ymd.png')


if __name__ == "__main__":
    main()
