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
    #beta = math.radians(1)  # degrees
    #delta = math.radians(0)  # degrees
    beta = 1
    delta = 0

    # define vehicle parameters
    wheelbase = 1000  # in milimetres [mm]
    trackwidth = 700  # in milimetres [mm]
    weightdist = .70  # percentage [%]
    mass = 250  # kilograms [kg]
    cornerstiff = 100

    # calculate vehicle characteristics
    a = wheelbase * weightdist / 1000
    b = wheelbase/1000 - a

    # solve for tire forces
    force_f = tire_model(beta+delta)
    force_r = tire_model(beta)

    f_lat = force_f + force_r
    a_lat = f_lat/mass
    yaw = a*force_f - b*force_r

    # develop matrix
    # TODO: better naming perhaps?
    a11 = cornerstiff*math.sin(delta)
    a12 = cornerstiff*math.sin(delta)
    a13 = cornerstiff
    a14 = cornerstiff
    a21 = cornerstiff*math.cos(delta)
    a22 = cornerstiff*math.cos(delta)
    a23 = 0
    a24 = 0
    a31 = cornerstiff*(trackwidth/1000/2*math.sin(delta) + a*math.cos(delta))
    a32 = cornerstiff*(-trackwidth/1000/2*math.sin(delta) + a*math.cos(delta))
    a33 = cornerstiff*-b
    a34 = cornerstiff*-b

    matrix = np.matrix([[a11, a12, a13, a14],
                        [a21, a22, a23, a24],
                        [a31, a32, a33, a34]])

    slipangles = np.matrix('1;1;1;1')

    result = matrix*slipangles
    print(result)

    print(f_lat)
    print(a_lat)
    print(yaw)

    print('I like turtles')
    x = np.array([1, 2, 3])
    print(x)
    force_fr = tire_model(2)
    print (force_fr)


if __name__ == "__main__":
    main()
