"""Simulator

Full 2-D simulation of rocket, including engine thrust,
drag, and gravity. Includes drag brakes controlled by a
closed-loop PID controller.
"""
import numpy as np
from matplotlib import pyplot as plt

from params import GRAVITY, MASS, SIM_TIME_INC, DRAG_FACTOR, DRAG_GAIN


class SimulatorState(object): # MUST extend object for py2 compat
    start_time = -5. # seconds

    error_values = []
    time_values = [start_time] # seconds
    altitude_values = [np.array([0., 0.])] # meters
    attitude_values = [0.] # radians
    velocity_values = [np.array([0., 0.])] # meters/second
    accel_values = [np.array([0., 0.])] # meters/seconds^2

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

