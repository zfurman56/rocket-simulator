"""Simulator

Full 2-D simulation of rocket, including engine thrust,
drag, and gravity. Includes drag brakes controlled by a
closed-loop PID controller.
"""
import numpy as np
from matplotlib import pyplot as plt

from params import GRAVITY, MASS, SIM_TIME_INC, DRAG_FACTOR, DRAG_GAIN
from kalman.kf import KalmanFilter


class SimulatorState(object): # MUST extend object for py2 compat
    _init_time = -5.
    _init_altitude = np.array([0., 0.])
    _init_attitude = 0.
    _init_velocity = np.array([0., 0.])
    _init_acceleration = np.array([0., 0.])

    def __init__(self):        
        # Use properties decorator to assert that the sim
        # has not started.
        self.init_time = property(lambda x: self._init_time, self.init_time)
        self.init_altitude = property(lambda x: self._init_altitude, self.init_altitude)
        self.init_attitude = property(lambda x: self._init_attitude, self.init_attitude)
        self.init_velocity = property(lambda x: self._init_velocity, self.init_velocity)
        self.init_acceleration = property(lambda x: self._init_acceleration, self.init_acceleration)

        self.time_values = [self._init_time] # seconds
        self.altitude_values = [self._init_altitude] # meters
        self.attitude_values = [self._init_attitude] # radians
        self.velocity_values = [self._init_velocity] # meters/second
        self.accel_values = [self._init_acceleration] # meters/seconds^2

    def print_init_values(self):
        # TODO check starting params exist
        print('===== INIT PARAMETERS ======')
        print('TIME::                   {} seconds'.format(self._init_time))
        print('ALTITUDE::               {} meters'.format(self._init_altitude))
        # print('ATTITUDE::               {} radians'.format(self._init_attitude))
        print('VELOCITY::               {} m/s'.format(self._init_velocity))
        print('ACCELERATION::           {} m/s^2'.format(self._init_acceleration))

    def _assert_init_values(self):
        assert len(self.time_values)    == 1 and \
            len(self.altitude_values)   == 1 and \
            len(self.attitude_values)   == 1 and \
            len(self.velocity_values)   == 1 and \
            len(self.accel_values)      == 1, """
            FATAL: either there was a continitiy error, the simulation
            has started, or an intial value is not properly configured.
            """

    def init_time(self, time):
        self._assert_init_values()
        self._init_time = time

    def init_altitude(self, altitude):
        self._assert_init_values()
        self._init_altitude = altitude

    def init_attitude(self, attitude):
        self._assert_init_values()
        self._init_attitude = attitude

    def init_velocity(self, velocity):
        self._assert_init_values()
        self._init_velocity = velocity

    def init_acceleration(self, accel):
        self._assert_init_values()
        self._init_acceleration = accel

    @property
    def time(self):
        return self.time_values[-1]

    @time.setter
    def time(self, time):
        self.time_values.append(time)

    @property
    def altitude(self):
        return self.altitude_values[-1]

    @altitude.setter
    def altitude(self, alt):
        self.altitude_values.append(alt)

    @property
    def attitude(self):
        raise NotImplementedError('Attitude has not been implemented yet.')

    @attitude.setter
    def attitude(self, att):
        raise NotImplementedError('Attitude has not been implemented yet.')

    @property
    def velocity(self):
        return self.velocity_values[-1]

    @velocity.setter
    def velocity(self, v):
        self.velocity_values.append(v)

    @property
    def acceleration(self):
        return self.accel_values[-1]

    @acceleration.setter
    def acceleration(self, a):
        self.accel_values.append(a)


class Simulator(SimulatorState):
    terminated = False

    def __init__(self):
        super(Simulator, self).__init__()
        dt = 0.00416666666

        self.kf = KalmanFilter(dim_x=3, dim_z=1)
        self.kf.x = np.array([0., 0., 0.])                                          # initial state (position, velocity, and acceleration)
        self.kf.F = np.array([[1., dt, 0.5*(dt**2)], [0., 1., dt], [0., 0., 1]])    # state transition matrix
        self.kf.H = np.array([[1., 0., 0.], [0., 0., 1.]])                          # Measurement function (baro)
        self.kf.H2 = np.array([[0., 0., 1.]])                                       # Measurement function (accel)
        self.kf.P *= 0.01                                                           # covariance matrix
        self.kf.R = np.array([[0.5, 0.], [0., 0.2]])                                # state uncertainty (baro)
        self.kf.R2 = np.array([[0.2]])                                              # state uncertainty (accel)
        self.kf.Q = np.array([[0.02, 0., 0.], [0., 0.02, 0.], [0., 0., 0.02]])

    def plot(self, title=None):
        plt.figure(title or 'Simulation Outcomes')
        plt.subplot(3, 1, 1)
        plt.plot(self.time_values, [x[1] for x in self.altitude_values], label='Altitude')
        plt.ylabel('Altitude (m)')

        plt.subplot(3, 1, 2)
        plt.plot(self.time_values, [x[1] for x in self.velocity_values], label='Velocity')
        plt.ylabel('Velocity (m/s)')

        plt.subplot(3, 1, 3)
        plt.plot(self.time_values, [x[1] for x in self.accel_values], label='Acceleration')
        plt.ylabel('Acceleration (m/s^2)')
        plt.xlabel('Time (s)')

        return plt

    def simulate(self):
        while not self.terminated:
            self.tick()
            # Increment the time
            self.time += SIM_TIME_INC
        self._print_results()

    def _print_results(self):
        print('Peak altitude (m):', max([x[1] for x in self.altitude_values]))
        print('Flight time (sec):', max(self.time_values))

    def _calculate_forces(self):
        # Add Engine thrust and attitude to forces
        # Gravitational force
        forces = MASS * GRAVITY
        # Air drag
        forces += DRAG_FACTOR * (1 + DRAG_GAIN) * -self.velocity**2 * np.sign(self.velocity)
        return forces

    def _terminate(self):
        return self.altitude[1] < 0

    def tick(self):
        if self._terminate():
            self.terminated = True

        if self.time < 0:
            # Duplicates the value on a list, ensuring equal lengths
            # for plotting
            self.altitude = self.altitude
            self.acceleration = self.acceleration
            self.velocity = self.velocity
            return

        # Calculate and set new acceleration, altitude, and velocity
        self.acceleration = self._calculate_forces() / MASS
        # Cannot use += due to property decorator
        self.altitude = self.altitude + self.velocity * SIM_TIME_INC + 0.5 * self.acceleration * SIM_TIME_INC**2
        self.velocity = self.velocity + self.acceleration * SIM_TIME_INC
