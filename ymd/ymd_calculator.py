import numpy as np
import math


def tire_model(alpha):
    """ returns tire force generated at a given slip angle
        input  - alpha (degree)
        output - lateral tire force (N)
    """
    # assume 600N/degree cornering stiffness
    return 100*alpha


def main():
    # define test conditions
    beta = math.radians(-10)
    delta = math.radians(0)

    # define vehicle parameters
    wheelbase = 1000  # in milimetres [mm]
    trackwidth = 700  # in milimetres [mm]
    weightdist = .70  # percentage [%]
    # mass = 250  # kilograms [kg]
    cornerstiff = -50

    # calculate vehicle characteristics
    a = wheelbase * weightdist / 1000
    b = wheelbase/1000 - a

    # develop matrix
    # TODO: better naming perhaps?
    a11 = cornerstiff*math.sin(delta)
    a12 = cornerstiff*math.sin(delta)
    a13 = 0
    a14 = 0
    a21 = cornerstiff*math.cos(delta)
    a22 = cornerstiff*math.cos(delta)
    a23 = cornerstiff
    a24 = cornerstiff
    a31 = cornerstiff*(trackwidth/1000/2*math.sin(delta) + a*math.cos(delta))
    a32 = cornerstiff*(-trackwidth/1000/2*math.sin(delta) + a*math.cos(delta))
    a33 = cornerstiff*-b
    a34 = cornerstiff*-b

    matrix = np.matrix([[a11, a12, a13, a14],   # F_x
                        [a21, a22, a23, a24],   # F_y
                        [a31, a32, a33, a34]])  # M_z

    # FR, FL, RR, RL
    slipangles = np.matrix([[beta+delta], [beta+delta], [beta], [beta]])

    result = matrix*slipangles
    print("Matrix Method - car model")
    print(result)


if __name__ == "__main__":
    main()
