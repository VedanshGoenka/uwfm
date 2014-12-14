import numpy as np


def tire_model(alpha):
    """ returns tire force generated at a given slip angle
        input  - alpha (degree)
        output - lateral tire force (N)
    """
    # assume 600N/degree cornering stiffness
    return 100*alpha


def main():
    # define test conditions
    beta = 9  # degrees
    delta = 0  # degrees

    # define vehicle parameters
    wheelbase = 1000  # in milimetres [mm]
    trackwidth = 700  # in milimetres [mm]
    weightdist = .50  # percentage [%]
    mass = 250  # kilograms [kg]

    # calculate vehicle characteristics
    a = wheelbase * weightdist / 1000
    b = wheelbase/1000 - a

    # solve for tire forces
    force_f = tire_model(beta+delta)
    force_r = tire_model(beta)

    f_lat = force_f + force_r
    a_lat = f_lat/mass
    yaw = a*force_f - b*force_r

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
