import configparser
import math
import numpy as np

from model.pacejka94 import Pacejka94 as pacejka


def main():
    # read in tire data
    config = configparser.ConfigParser()
    config.read('data/tire/hoosier_lc0.ini')
    coeff = {key: float(config['lateral'][key]) for key in config['lateral']}

    # build tire model
    tire_model = pacejka(coeff)

    alpha_sweep = np.arange(-20, 21)

    force_y = [tire_model.fy(22.7*9.81, math.radians(alpha), 0) for alpha in alpha_sweep]

    print(force_y)


if __name__ == "__main__":
    main()
