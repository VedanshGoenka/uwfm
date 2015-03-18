import configparser
import math
import matplotlib.pyplot as plt
import numpy as np

from model.pacejka_MF52 import PacejkaMF52 as pacejka


def main():
    # read in tire data
    config = configparser.ConfigParser()
    config.read('data/tire/hoosier_lc0_test.ini')
    coeff = {key: float(config['lateral'][key]) for key in config['lateral']}

    # build tire model
    tire_model = pacejka(coeff)

    alpha_sweep = np.arange(-20, 21)

    force_y = [tire_model.calc_fy(-22*9.81*3, math.radians(alpha), 0, 0) for alpha in alpha_sweep]
    mom_z = [tire_model.calc_mz(-22*9.81*3, math.radians(alpha), 0, 0) for alpha in alpha_sweep]

    print(force_y)
    print(mom_z)

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
