import configparser
import math
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np

from model.Pacejka_MF52 import PacejkaMF52 as pacejka


def main():
    # read in tire data
    config = configparser.ConfigParser(inline_comment_prefixes=';')
    config.read('data/tire/hoosier_lc0_sorted.ini')

    # Extract coefficients
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

    # build tire model
    tire_model = pacejka(general, u, py, qz, px, qx, rx, ry, sz, qy)

    alpha_sweep = np.arange(-20, 20, 0.5)
    kappa_sweep = np.arange(-0.3, 0.3, 0.0175)

    force_x = [tire_model.calc_fx(-22*9.81*2, 0, kappa, 0) for kappa in kappa_sweep]
    force_y = [tire_model.calc_fy(-22*9.81*2, math.radians(alpha), 0, 0) for alpha in alpha_sweep]
    force_y2 = [tire_model.calc_fy(-22*9.81*2, math.radians(alpha), 0,
        math.radians(-20)) for alpha in alpha_sweep]
    mom_z = [tire_model.calc_mz(-22*9.81*2, math.radians(alpha), 0, 0) for alpha in alpha_sweep]

    # Plotting
    plt.figure()
    plt.plot(kappa_sweep, force_x)
    plt.grid()
    plt.title('Longitudinal Force vs. Slip Ratio - 100 lbs')
    plt.xlabel('Slip Ratio [%]')
    plt.ylabel('Longitudinal Force [N]')

    plt.figure()
    plt.plot(alpha_sweep, force_y)
    # plt.plot(alpha_sweep, force_y2)
    plt.grid()
    plt.title('Lateral Force vs. Slip Angle - 100 lbs')
    plt.xlabel('Slip Angle [deg]')
    plt.ylabel('Lateral Force [N]')

    # label1 = mpatches.Patch(label='IA 0 degrees')
    # label2 = mpatches.Patch(color='green', label='IA -20 degrees')
    # plt.legend(handles=[label1, label2], prop={'size': 12})

    plt.figure()
    plt.plot(alpha_sweep, mom_z)
    plt.grid()
    plt.title('Self-aligning Torque vs. Slip Angle - 100 lbs')
    plt.xlabel('Slip Angle [deg]')
    plt.ylabel('Yaw Moment [Nm]')

    # Make a friction elipse
    force_x = [[tire_model.calc_fx(-22*9.81*2, math.radians(alpha),
        kappa, 0) for kappa in kappa_sweep] for alpha in alpha_sweep]
    force_y = [[tire_model.calc_fy(-22*9.81*2, math.radians(alpha),
        kappa, 0) for kappa in kappa_sweep] for alpha in alpha_sweep]

    plt.figure()
    plt.plot(force_y, force_x)
    plt.plot(np.transpose(force_y), np.transpose(force_x))

    plt.show()

if __name__ == "__main__":
    main()
