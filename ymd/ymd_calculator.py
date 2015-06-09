import configparser
import math
import matplotlib.pyplot as plt
import numpy as np

from model.Pacejka_MF52 import PacejkaMF52 as Pacejka
from model.Vehicle import Vehicle
from model.YMDSimulation import YMDSimulation


def build_tire_model(filename):
    '''Builds a tire model instance using parameters in filename'''

    config = configparser.ConfigParser(inline_comment_prefixes=';')
    config.read(filename)

    general = {key: float(config['general'][key]) for key in config['general']}
    py = {key: float(config['lateral'][key]) for key in config['lateral']}
    qz = {key: float(config['self_aligning'][key]) for key in config['self_aligning']}
    px = {key: float(config['longitudinal'][key]) for key in config['longitudinal']}
    qx = {key: float(config['overturning'][key]) for key in config['overturning']}
    rx = {key: float(config['longitudinal_combined'][key]) for key in config['longitudinal_combined']}
    ry = {key: float(config['lateral_combined'][key]) for key in config['lateral_combined']}
    sz = {key: float(config['self_aligning_combined'][key]) for key in config['self_aligning_combined']}
    qy = {key: float(config['roll_moment'][key]) for key in config['roll_moment']}

    return Pacejka(general, py, qz, px, qx, rx, ry, sz, qy)


def build_vehicle_model(vehicle_config, tire_config):
    '''Builds a vehicle model instance using parameters in vehicle_config and tire_config'''

    config = configparser.ConfigParser(inline_comment_prefixes=';')
    config.read(vehicle_config)

    dimensions = {key: float(config['dimensions'][key]) for key in config['dimensions']}

    # TODO: allow flexible way to specify tires on all tires
    tire_model = build_tire_model(tire_config)

    return Vehicle(tire_model, tire_model, tire_model, tire_model, dimensions)


def build_simulation_params(simulation_config):
    '''Builds a YMDSimulation instance to store YMD simulation paramaters'''

    config = configparser.ConfigParser(inline_comment_prefixes=';')
    config.read(simulation_config)

    simulation_params = {key: float(config['simulation'][key]) for key in config['simulation']}

    return YMDSimulation(simulation_params)


def converge_lateral_acceleration(vehicle_model, simulation, beta, delta, relaxation):
        # Initialize variables
        m_z = 0
        a_lat = 0
        a_lat_prev = 100

        # Iterate until we are within the allowed tolerance
        while math.fabs(a_lat - a_lat_prev) > simulation.tolerance:
            # Relaxation to allow convergence at low speeds
            a_lat = a_lat * relaxation + a_lat_prev * (1 - relaxation)
            a_lat_prev = a_lat

            # Calculate vehicle yaw speed
            yaw_speed = a_lat/simulation.velocity

            solution = car.calc_vehicle_forces(simulation.velocity, yaw_speed, a_lat, beta, delta)

            a_lat = solution.flat[1]/car.mass
            m_z = solution.flat[2]

        return solution


def main():
    # define test conditions
    simulation = build_simulation_params('data/simulation/generic_ymd.ini')
    car = build_vehicle_model('data/vehicle/generic_fsae.ini', 'data/tire/hoosier_lc0_sorted.ini')

    # Initialize results arraw
    result_ay = np.empty([len(simulation.beta_range), len(simulation.delta_range)])
    result_mz = np.empty([len(simulation.beta_range), len(simulation.delta_range)])

    for i, beta in enumerate(simulation.beta_range):
        for j, delta in enumerate(simulation.delta_range):
            # Set static tire load
            car.a_lat = 0
            car.a_long = 0

            car.delta = delta

            # Initialize variables
            m_z = 0
            a_lat = 0
            a_lat_prev = 100
            relaxation = 0.5  # TODO: Move me somewhere else!

            while math.fabs(a_lat - a_lat_prev) > simulation.tolerance:
                # Relaxation to allow convergence at low speeds
                a_lat = a_lat * relaxation + a_lat_prev * (1 - relaxation)
                a_lat_prev = a_lat

                # Calculate vehicle yaw speed
                yaw_speed = a_lat/simulation.velocity

                solution = car.calc_vehicle_forces(simulation.velocity, yaw_speed, a_lat, beta, delta)

                a_lat = solution.flat[1]/car.mass
                m_z = solution.flat[2]

            result_ay[i][j] = a_lat / 9.81
            result_mz[i][j] = m_z

    # Plot
    plt.plot(np.transpose(result_ay)[:][:],
             np.transpose(result_mz)[:][:], color='black')
    plt.plot(result_ay[:][:], result_mz[:][:], color='red')
    plt.xlabel('Lateral Acceleration [g]')
    plt.ylabel('Yaw Moment [Nm]')
    plt.title('Yaw Moment Diagram - {:.1f} [m/s]'.format(simulation.velocity))
    plt.grid(True)
    plt.show()
    plt.savefig('ymd.png')


if __name__ == "__main__":
    main()
