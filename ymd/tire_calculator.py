import configparser
import math
import matplotlib.pyplot as plt
import numpy as np

from model.Pacejka_MF52 import PacejkaMF52 as pacejka


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
    tire_model = pacejka(general, py, qz, px, qx, rx, ry, sz, qy)

    alpha_sweep = np.arange(-20, 21)

    force_y = [tire_model.calc_fy(-22*9.81*3, math.radians(alpha), 0, 0) for alpha in alpha_sweep]
    mom_z = [tire_model.calc_mz(-22*9.81*3, math.radians(alpha), 0, 0) for alpha in alpha_sweep]

    # Result
    print(force_y)
    print(mom_z)

    # Plotting
    plt.figure()
    plt.plot(alpha_sweep, force_y)
    plt.grid()
    plt.xlabel('Slip Angle [deg]')
    plt.ylabel('Lateral Force [N]')
    plt.savefig('latforce.png')

    plt.figure()
    plt.plot(alpha_sweep, mom_z)
    plt.grid()
    plt.xlabel('Slip Angle [deg]')
    plt.ylabel('Yaw Moment [Nm]')
    plt.show()
    plt.savefig('yawmom.png')

if __name__ == "__main__":
    main()
