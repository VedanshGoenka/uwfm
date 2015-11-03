#!/usr/bin/env python3

import numpy as np
import scipy.signal as sp
import matplotlib.pyplot as plt

# Let mass2 be the sprung mass
# Let mass1 be the unsprung mass
# Let spring2 and damper2 be between the unsprung and sprung mass
# Let spring1 be between the unsprung mass and the road

# Define some of our model parameters
mass2 = 50  # [kg]
mass1 = 9   # [kg]
spring2 = 4160  # [N/m]
spring1 = 168307  # [N/m] Hoosier R25B
damper2 = 1400  # [N/m/s]

# For road excitation
char_poly = [mass1 * mass2,
             mass1 * damper2 + mass2 * damper2,
             mass1 * spring2 + mass2 * spring1 + mass2 * spring2,
             damper2 * spring1,
             spring1 * spring2]

sprung_num = [spring1 * damper2, spring1 * spring2]
sprung_sys = sp.TransferFunction(sprung_num, char_poly)

unsprung_num = [mass1 * spring1, damper2 * spring1, spring1 * spring2]
unsprung_sys = sp.TransferFunction(unsprung_num, char_poly)

# Generate a road input. Let us do a step input for fun!
road_input = 0.1 * np.ones(10000)
time = np.linspace(0, 2, 10000)

# Simulate the motion of the masses. I have no idea what the third output is
tout, sprung_resp, x = sp.lsim(sprung_sys, road_input, time)
tout, unsprung_resp, x = sp.lsim(unsprung_sys, road_input, time)

# Plot
plt.plot(tout, sprung_resp)
plt.plot(tout, unsprung_resp)
plt.show()
