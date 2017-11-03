"""Constants and settings used throught the simulator.
"""
import numpy as np
# from scipy.constants import g

GRAVITY = np.array([0, -9.8]) # TODO use scipy.constants for gravity
BARO_STD = 0.3  # Baro altitude sensor stddev (m)
ACCEL_STD = 0.1  # Accelerometer sensor stddev (m/s^2)
MAX_SERVO_SLEW_RATE = np.pi * 2 # rad/s
MASS = 0.625 #kilograms
TARGET_APOGEE = 236 # meters
ROD_LEN = 0
STEP_SIZE = 0.00416666666 #seconds
CMD_PERIOD = 0.05 #seconds
KP = 0.0008
KI = 6
KD = 0.0
DRAG_FACTOR = 0.00185
ES_DRAG_FACTOR = 0.00185
DRAG_GAIN = 10
THRUST_SCALE = 1.0
