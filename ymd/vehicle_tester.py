import math
import configparser
import numpy as np

from model.pacejka_MF52 import PacejkaMF52 as Pacejka
from model.Vehicle import Vehicle

def main():
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
    tire_model = Pacejka(general, py, qz, px, qx, rx, ry, sz, qy)

    # read in vehicle data
    config.read('data/vehicle/generic_fsae.ini')
    dimensions = {key: float(config['dimensions'][key]) for key in config['dimensions']}

    # build vehicle class
    car = Vehicle(tire_model, tire_model, tire_model, tire_model,
            dimensions)

    # do some basic tasks
    car.velocity = 50/3.6

    delta_sweep = np.arange(-20,20)  # in degrees
    forces = [car.resolve_forces(math.radians(delta)).flat[1] for delta in delta_sweep]

    import pdb; pdb.set_trace()


if __name__ == "__main__":
    main()
