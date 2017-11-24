"""
Constants and settings used throught the simulator.

"""

import numpy as np
# from scipy.constants import g


GRAVITY = np.array([0, -9.8]) # TODO use scipy.constants for gravity

# Barometer sensor standard deviation
# Randomizes 
BARO_STD = 0.3  # meters

# Accelerometer sensor standar deviation
# Randomizes
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

DRAG_FACTOR = 0.00185

DRAG_GAIN = 10

THRUST_SCALE = 1.0

# Length of the aluminium launch rod
# Helps with rocket rotation
ROD_LEN = 0
