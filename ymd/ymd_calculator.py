import argparse
import configparser
import math
import matplotlib.pyplot as plt
import numpy as np

from model.Pacejka_MF52 import PacejkaMF52 as Pacejka
from model.Vehicle import Quartet
from model.Vehicle import Vehicle
from model.YMDSimulation import YMDSimulation


def build_tire_model(filename):
    '''Builds a tire model instance using parameters in filename'''

    config = configparser.ConfigParser(inline_comment_prefixes=';')
    config.read(filename)

    general = {key: float(config['general'][key]) for key in config['general']}
    u = {key: float(config['scaling'][key]) for key in config['scaling']}
    py = {key: float(config['lateral'][key]) for key in config['lateral']}
    qz = {key: float(config['self_aligning'][key]) for key in config['self_aligning']}
    px = {key: float(config['longitudinal'][key]) for key in config['longitudinal']}
    qx = {key: float(config['overturning'][key]) for key in config['overturning']}
    rx = {key: float(config['longitudinal_combined'][key]) for key in config['longitudinal_combined']}
    ry = {key: float(config['lateral_combined'][key]) for key in config['lateral_combined']}
    sz = {key: float(config['self_aligning_combined'][key]) for key in config['self_aligning_combined']}
    qy = {key: float(config['roll_moment'][key]) for key in config['roll_moment']}

    return Pacejka(general, u, py, qz, px, qx, rx, ry, sz, qy)


def build_vehicle_model(vehicle_config, tire_config):
    '''Builds a vehicle model instance using parameters in vehicle_config and tire_config'''

    config = configparser.ConfigParser(inline_comment_prefixes=';')
    config.read(vehicle_config)

    mass = {key: float(config['mass'][key]) for key in config['mass']}
    geometry = {key: float(config['geometry'][key]) for key in config['geometry']}
    suspension = {key: float(config['suspension'][key]) for key in config['suspension']}
    aero = {key: float(config['aerodynamic'][key]) for key in config['aerodynamic']}
    setup = {key: float(config['setup'][key]) for key in config['setup']}

    tire_model = build_tire_model(tire_config)
    tires = Quartet(tire_model, tire_model, tire_model, tire_model)  # same tires on all four corners

    return Vehicle(tires, mass, geometry, suspension, aero, setup)


def build_simulation_params(simulation_config):
    '''Builds a YMDSimulation instance to store YMD simulation paramaters'''

    config = configparser.ConfigParser(inline_comment_prefixes=';')
    config.read(simulation_config)

    simulation_params = {key: float(config['simulation'][key]) for key in config['simulation']}

    return YMDSimulation(simulation_params)


def converge_lateral_acceleration(car, simulation, beta, delta):
    '''Iterates vehicle model until the lateral acceleration converges'''

    # Initialize variables
    a_lat = 0
    a_lat_prev = 100

    # Iterate until we are within the allowed tolerance
    while math.fabs(a_lat - a_lat_prev) > simulation.tolerance:
        # Relaxation to allow convergence at low speeds
        a_lat = a_lat * simulation.relaxation + a_lat_prev * (1 - simulation.relaxation)
        a_lat_prev = a_lat

        # Calculate vehicle yaw speed
        yaw_speed = a_lat/simulation.velocity

        solution = car.calc_vehicle_forces(simulation.velocity, yaw_speed, a_lat, beta, delta)

        a_lat = solution.flat[1]/car.mass

    return solution


def plot_ymd_results(result_ay, result_mz, simulation):
    plt.plot(np.transpose(result_ay)[:][:],
             np.transpose(result_mz)[:][:], color='black')
    plt.plot(result_ay[:][:], result_mz[:][:], color='red')

    plt.xlabel('Lateral Acceleration [g]')
    plt.ylabel('Yaw Moment [Nm]')
    plt.title('Yaw Moment Diagram - {:.1f} [m/s]'.format(simulation.velocity))
    plt.grid(True)

    plt.show()


def ymd_calculator(car, simulation):
    '''Generates a Yaw Moment Diagram using the given parameters in the config files'''

    # Initialize results arraw
    result_ay = np.empty([len(simulation.beta_range), len(simulation.delta_range)])
    result_mz = np.empty([len(simulation.beta_range), len(simulation.delta_range)])

    # Sweep through all the combinations of beta and delta and solve
    for i, beta in enumerate(simulation.beta_range):
        for j, delta in enumerate(simulation.delta_range):
            solution = converge_lateral_acceleration(car, simulation, beta, delta)

            result_ay[i][j] = solution.flat[1]/car.mass / 9.81
            result_mz[i][j] = solution.flat[2]

    return result_ay, result_mz


def ymd_calculator_wrapper(simulation_config, vehicle_config, tire_config):
    '''Wrapper function that builds the models for us before calling ymd_calculator'''
    simulation = build_simulation_params(simulation_config)
    car = build_vehicle_model(vehicle_config, tire_config)

    result_ay, result_mz = ymd_calculator(car, simulation)

    return result_ay, result_mz, car, simulation


def main():
    parser = argparse.ArgumentParser(description='Generates a Yaw Moment Diagram')
    parser.add_argument('-v', '--vehicle-config', type=str, required=True)
    parser.add_argument('-t', '--tire-config', type=str, required=True)
    parser.add_argument('-s', '--simulation-config', type=str, required=True)
    parser.add_argument('-o', '--output', type=str, required=False)
    args = parser.parse_args()

    results_ay, results_mz, vehicle_model, ymd_simulation = ymd_calculator_wrapper(args.simulation_config, args.vehicle_config, args.tire_config)

    plot_ymd_results(results_ay, results_mz, ymd_simulation)


if __name__ == "__main__":
    main()
