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

    alpha_sweep = np.arange(-13, 14)

    force_y = [tire_model.calc_fy(-22*9.81*4, math.radians(alpha), 0,
        0.035) for alpha in alpha_sweep]

    print(force_y)

    plt.plot(alpha_sweep, force_y)
    plt.grid()
    plt.savefig('latforce.png')

if __name__ == "__main__":
    main()
