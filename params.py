"""
Constants and settings used throught the simulator.

"""

import numpy as np
from scipy.constants import g


GRAVITY = np.array([0, -g])

# Barometer sensor standard deviation
# Adds some randomness when normalized
BARO_STD = 0.3  # meters

# Accelerometer sensor standar deviation
# Adds some randomness when normalized
ACCEL_STD = 0.1  # meters/second^2

MAX_SERVO_SLEW_RATE = np.pi * 2 # rad/second

# Mass of the rocket. Used to calculate forces.
MASS = 0.625 # kilograms

TARGET_APOGEE = 236 # meters

# How much time passes between each tick
SIM_TIME_INC = 0.00416666666 # seconds

CMD_PERIOD = 0.05 # seconds

## PID
# Proportional Gain
KP = 0.008
# Integral Gain
KI = 0.0
# Derivative Gain
KD = 0.0

# Drag factor for rocket
# 0.5 * Cd  * A * rho
# To simulate, solve for Cd*A*rho using the terminal velocity equation
# and multiply by 0.5
DRAG_FACTOR = 0.00185

DRAG_GAIN = 10

THRUST_SCALE = 1.0

# Length of the aluminium launch rod
# Helps with rocket rotation
ROD_LEN = 0

# Chute terminal velocity
CHUTE_TERM_VEL = 5. # m/s

# Chute drag factor, using derived terminal velocity
CHUTE_DRAG_FACTOR = (MASS * -GRAVITY[1]) / CHUTE_TERM_VEL**2

# CHUTE_APEX_VENT_RAD = 0.03 # m
# CHUTE_CANOPY_RAD = 0.08 # m
# EST_CHUTE_DRAG_FACTOR = (0.5 *
#                      1.2041 * # rho @ 1atm and 20C - https://en.wikipedia.org/wiki/Density_of_air
#                      1.75 * # typical Cd - see https://www.grc.nasa.gov/WWW/k-12/VirtualAero/BottleRocket/airplane/rktvrecv.html
#                      (np.pi * CHUTE_CANOPY_RAD**2 - np.pi * CHUTE_APEX_VENT_RAD**2) # Cross sectional area of chute
#                     )