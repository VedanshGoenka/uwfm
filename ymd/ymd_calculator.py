import argparse
import configparser
import math
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np

from model.Pacejka_MF52 import PacejkaMF52 as Pacejka
from model.Vehicle import Quartet
from model.Vehicle import Vehicle
from model.YawMomentDiagram import YMDParameters, YMDSimulation, YMDAnalysis


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

    return YMDParameters(simulation_params)


def main():
    # Parse input arguments
    parser = argparse.ArgumentParser(description='Generates a Yaw Moment Diagram')
    parser.add_argument('-v', '--vehicle-config', type=str, required=True)
    parser.add_argument('-t', '--tire-config', type=str, required=True)
    parser.add_argument('-s', '--simulation-config', type=str, required=True)
    parser.add_argument('-o', '--output', type=str, required=False)
    args = parser.parse_args()

    # Get the vehicle 
    car = build_vehicle_model(args.vehicle_config, args.tire_config)

    # Build the simulation parameter instance
    parameters = build_simulation_params(args.simulation_config)

    # Build the simulation instance
    simulation = YMDSimulation(car, parameters)

    # Run the simulation
    simulation.start_simulation()

    # Build a YMDAnalysis instance
    analysis = YMDAnalysis(simulation)

    # Output metrics
    print(analysis.max_lateral_accel)
    print(analysis.residual_yaw_moment)
    print(analysis.trim_lateral_accel)
    print(analysis.entry_control)
    print(analysis.entry_stability)

    # Plot the results
    analysis.plot_results()
    analysis.plot_nondim_results()
    analysis.show_plots()

if __name__ == "__main__":
    main()
