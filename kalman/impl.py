import numpy as np
from main import KalmanFilter

dt = 0.00416666666

kfilter = KalmanFilter(dim_x=3, dim_z=1)

kfilter.x = np.array([0., 0., 0.])            # initial state (position, velocity, and acceleration)

kfilter.F = np.array([[1., dt, 0.5*(dt**2)],
                      [0., 1., dt],
                      [0., 0., 1]])     # state transition matrix

kfilter.H = np.array([[1., 0., 0.],
                      [0., 0., 1.]])    # Measurement function (baro)

kfilter.H2 = np.array([[0., 0., 1.]])   # Measurement function (accel)

kfilter.P *= 0.01                         # covariance matrix

kfilter.R = np.array([[0.5, 0.],
                      [0., 0.2]])               # state uncertainty (baro)

kfilter.R2 = np.array([[0.2]])               # state uncertainty (accel)

kfilter.Q = np.array([[0.02, 0., 0.],
                      [0., 0.02, 0.],
                      [0., 0., 0.02]])
