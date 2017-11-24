"""Simulator

Full 2-D simulation of rocket, including engine thrust,
drag, and gravity. Includes drag brakes controlled by a
closed-loop PID controller.
"""
import numpy as np
from matplotlib import pyplot as plt

from params import GRAVITY, MASS, SIM_TIME_INC, DRAG_FACTOR, DRAG_GAIN, ROD_LEN
from kalman.kf import KalmanFilter


class SimulatorState(object): # MUST extend object for py2 compat
    time_values = [-5.] # seconds
    altitude_values = [np.array([0., 0.])] # meters
    # See https://spaceflightsystems.grc.nasa.gov/education/rocket/rotations.html
    # for more information about attitude of rocket
    pitch_values = [0.] # radians
    yaw_values = [0.] # radians
    roll_values = [0.] # radians
    velocity_values = [np.array([0., 0.])] # meters/second
    acceleration_values = [np.array([0., 0.])] # meters/seconds^2

    def print_init_values(self):
        print('')
        print('===== INIT PARAMETERS ======')
        print('TIME::                   {} seconds'.format(self.time_values[0]))
        print('ALTITUDE::               {} meters'.format(self.altitude_values[0]))
        print('PITCH::                  {} radians'.format(self.pitch_values[0]))
        print('YAW::                    {} radians'.format(self.yaw_values[0]))
        print('ROLL::                   {} radians'.format(self.roll_values[0]))
        print('VELOCITY::               {} m/s'.format(self.velocity_values[0]))
        print('ACCELERATION::           {} m/s^2'.format(self.acceleration_values[0]))
        print('============================')
        print('')

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
    def pitch(self):
        return self.pitch_values[-1]

    @pitch.setter
    def pitch(self, ptc):
        self.pitch_values.append(ptc)

    @property
    def yaw(self):
        return self.yaw_values[-1]

    @yaw.setter
    def yaw(self, yaw):
        self.yaw_values.append(yaw)

    @property
    def roll(self):
        return self.roll_values[-1]

    @roll.setter
    def roll(self, roll):
        self.roll_values.append(roll)

    @property
    def velocity(self):
        return self.velocity_values[-1]

    @velocity.setter
    def velocity(self, v):
        self.velocity_values.append(v)

    @property
    def acceleration(self):
        return self.acceleration_values[-1]

    @acceleration.setter
    def acceleration(self, a):
        self.acceleration_values.append(a)


class Simulator(SimulatorState):
    terminated = False

    def print_final_values(self):
        assert self.terminated, 'The simulation has not terminated; cannot print final values.'

        print('')
        print('===== FINAL PARAMETERS ======')
        print('TIME::                   {} seconds'.format(self.time_values[-1]))
        print('ALTITUDE::               {} meters'.format(self.altitude_values[-1]))
        print('PITCH::                  {} radians'.format(self.pitch_values[-1]))
        print('YAW::                    {} radians'.format(self.yaw_values[-1]))
        print('ROLL::                   {} radians'.format(self.roll_values[-1]))
        print('VELOCITY::               {} m/s'.format(self.velocity_values[-1]))
        print('ACCELERATION::           {} m/s^2'.format(self.acceleration_values[-1]))
        print('=============================')
        print('')

    def plot(self, title=None):
        plt.figure(title or 'Simulation Outcomes')
        plt.subplot(3, 1, 1)
        plt.plot(self.time_values, [x[1] for x in self.altitude_values], label='Altitude')
        plt.ylabel('Altitude (m)')

        plt.subplot(3, 1, 2)
        plt.plot(self.time_values, [x[1] for x in self.velocity_values], label='Velocity')
        plt.ylabel('Velocity (m/s)')

        plt.subplot(3, 1, 3)
        plt.plot(self.time_values, [x[1] for x in self.acceleration_values], label='Acceleration')
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
        # Add Engine thrust and pitch to forces
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

        if np.linalg.norm(self.altitude) > ROD_LEN:
            self.pitch = np.pi/2 - np.arctan2(self.velocity[1], self.velocity[0])


class KFSimulator(Simulator):
    kalman_altitude_values = [0.] # meters
    kalman_velocity_values = [0.] # meters/second
    kalman_acceleration_values = [0.] # meters/second^2

    def __init__(self):
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

    @property
    def estimated_altitude(self):
        return self.kalman_altitude_values[-1]

    @estimated_altitude.setter
    def estimated_altitude(self, alt):
        self.kalman_altitude_values.append(alt)

    @property
    def estimated_velocity(self):
        return self.kalman_velocity_values[-1]

    @estimated_velocity.setter
    def estimated_velocity(self, v):
        self.kalman_velocity_values.append(v)

    @property
    def estimated_acceleration(self):
        return self.kalman_acceleration_values[-1]

    @estimated_acceleration.setter
    def estimated_acceleration(self, a):
        self.kalman_acceleration_values.append(a)
