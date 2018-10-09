"""Simulates Apogee Control by use of drag brakes.

"""
from copy import deepcopy

import numpy as np
from matplotlib import pyplot as plt

from simulator import KFState, Simulator

from apogee_ctrl import ApogeeSimulator
from utils import get_eng_file_from_argv

from pid import PIDController
from params import (
    GRAVITY,
    MAX_SERVO_SLEW_RATE,
    MASS,
    TARGET_APOGEE,
    SIM_TIME_INC,
    CMD_PERIOD,
    KP, KI, KD,
    CHUTE_DRAG_FACTOR,
    DRAG_GAIN,
    ROD_LEN,
    TARGET_DURATION,
    CHUTE_TERM_VEL,
    DESCENT_STR_LEN
)


class DescentSimulator(Simulator):
    class DescentState(KFState):
        def __init__(self, copy=None, apogee=None):
            super(DescentSimulator.DescentState, self).__init__()

            self.chute_deployed = False
            self.pin_pulled = False

            if isinstance(copy, KFState): #copy contructor
                self.__dict__.update(copy.__dict__)
                # Chute init data
                self.chute_altitude_values = self.altitude_values # meters
                self.chute_velocity_values = self.velocity_values # meters/second
                self.chute_acceleration_values = self.acceleration_values # meters/seconds^2
            else:
                # kalman filter
                self.kf.x = [apogee or TARGET_APOGEE, # altitude - meters
                            0., # velocity - m/s
                            GRAVITY[1]] # acceleration - m/s^2

                self.kalman_altitude_values = [apogee or TARGET_APOGEE]
                self.kalman_velocity_values = [0.]
                self.kalman_acceleration_values = [GRAVITY[1]]

                # Rocket init data
                self.altitude_values = [apogee or np.array([0., TARGET_APOGEE])] # meters
                self.velocity_values = [np.array([0., 0.])] #m/s
                self.acceleration_values = [GRAVITY] # m/s^2
                self.time_values = [6.3] # seconds

                # Chute init data
                self.chute_altitude_values = [apogee or np.array([0., TARGET_APOGEE])] # meters
                self.chute_velocity_values = [np.array([0., 0.])] # meters/second
                self.chute_acceleration_values = [GRAVITY] # meters/seconds^2

        @property
        def chute_altitude(self):
            return self.chute_altitude_values[-1]

        @chute_altitude.setter
        def chute_altitude(self, alt):
            self.chute_altitude_values.append(alt)

        @property
        def chute_velocity(self):
            return self.chute_velocity_values[-1]

        @chute_velocity.setter
        def chute_velocity(self, v):
            self.chute_velocity_values.append(v)

        @property
        def chute_acceleration(self):
            return self.chute_acceleration_values[-1]

        @chute_acceleration.setter
        def chute_acceleration(self, a):
            self.chute_acceleration_values.append(a)


    def __init__(self, state=None):
        super(DescentSimulator, self).__init__(state or DescentSimulator.DescentState())
        assert isinstance(self.state, self.DescentState), 'State must be of type `DescentState` or subclass for descent control simulation.'
        self.pid = PIDController(TARGET_APOGEE, KP, KI, KD)

    def simulate(self):
        while not self.terminated:
            if self.state.altitude[1] / CHUTE_TERM_VEL < TARGET_DURATION - self.state.time:
                self.state.chute_deployed = True

            # Deploys only when in range of 15m to 10m above ground
            if self.state.chute_deployed and (15. > self.state.altitude[1] > 10.) and (self.state.altitude[1] - DESCENT_STR_LEN) / CHUTE_TERM_VEL > TARGET_DURATION - self.state.time:
                self.state.pin_pulled = True

            # TODO rewrite this portion; slight messy.
            # Prevents floating point errors because SIM_TIME_INC is small.
            sim_time_end = self.state.time + CMD_PERIOD - SIM_TIME_INC/2
            first = True
            while self.state.time < sim_time_end:
                self.tick()

                self.state.kf.predict()
                if first:
                    self.state.kf.update(np.array([self._altimeter_model(self.state.altitude), self._accelerometer_model(-self.state.acceleration) - GRAVITY[1]]))
                else:
                    self.state.kf.update2(np.array([self._accelerometer_model(-self.state.acceleration) - GRAVITY[1]]))

                first = False

                # Appends values to lists
                self.state.estimated_altitude = self.state.kf.x[0]
                self.state.estimated_velocity = self.state.kf.x[1]
                self.state.estimated_acceleration = self.state.kf.x[2]

                # Increment the time
                self.state.time += SIM_TIME_INC

        self._print_results()

    def tick(self):
        if self._terminate():
            self.terminated = True

        if self.state.time < 0:
            # Duplicates the value on a list, ensuring equal lengths
            # for plotting
            self.state.altitude = self.state.altitude
            self.state.acceleration = self.state.acceleration
            self.state.velocity = self.state.velocity

            self.state.chute_altitude = self.state.chute_altitude
            self.state.chute_acceleration = self.state.chute_acceleration
            self.state.chute_velocity = self.state.chute_velocity
            return

        # Calculate and set new acceleration, altitude, and velocity
        self.state.acceleration = self._calculate_forces() / MASS
        self.state.chute_acceleration = self._calculate_chute_forces() / MASS
        # Cannot use += due to property decorator
        self.state.altitude = self.state.altitude + self.state.velocity * SIM_TIME_INC + 0.5 * self.state.acceleration * SIM_TIME_INC**2
        self.state.velocity = self.state.velocity + self.state.acceleration * SIM_TIME_INC
        self.state.chute_altitude = self.state.chute_altitude + self.state.chute_velocity * SIM_TIME_INC + 0.5 * self.state.chute_acceleration * SIM_TIME_INC**2
        self.state.chute_velocity = self.state.chute_velocity + self.state.chute_acceleration * SIM_TIME_INC

        if np.linalg.norm(self.state.altitude) > ROD_LEN:
            self.state.pitch = np.pi/2 - np.arctan2(self.state.velocity[1], self.state.velocity[0])

    def _calculate_forces(self):
        # Assumes drag brakes are not in use
        if not self.state.chute_deployed or (self.state.pin_pulled and self.state.altitude[1] + DESCENT_STR_LEN > self.state.chute_altitude[1]):
            return super(DescentSimulator, self)._calculate_forces()
        forces = MASS * GRAVITY
        forces += CHUTE_DRAG_FACTOR * -self.state.velocity**2 * np.sign(self.state.velocity)
        return forces

    def _calculate_chute_forces(self):
        if not self.state.chute_deployed:
            return super(DescentSimulator, self)._calculate_forces()
        forces = MASS * GRAVITY
        forces += CHUTE_DRAG_FACTOR * -self.state.chute_velocity**2 * np.sign(self.state.chute_velocity)
        return forces

    def _print_results(self):
        print('')
        print('========== RESULTS ==========')
        print('TIME::                   {} seconds'.format(max(self.state.time_values)))
        print('TARGET TIME::            44.5 ± 1.5 seconds')
        print('=============================')
        print('')

    def _terminate(self):
        return self.state.altitude[1] < 0

    def plot(self, title=None):
        kf_label = 'KF estimation'

        plt.figure(title or 'Simulation Outcomes')
        plt.subplot(4, 1, 1)
        plt.plot(self.state.time_values, [x[1] for x in self.state.altitude_values], label='Altitude')
        plt.plot(self.state.time_values, [x[1] for x in self.state.chute_altitude_values], label='Chute Altitude')
        plt.plot(self.state.time_values, self.state.kalman_altitude_values, label=kf_label)
        plt.ylabel('Altitude (m)')
        plt.legend()

        plt.subplot(4, 1, 2)
        plt.plot(self.state.time_values, [x[1] for x in self.state.velocity_values], label='Velocity')
        plt.plot(self.state.time_values, [x[1] for x in self.state.chute_velocity_values], label='Chute Velocity')
        plt.plot(self.state.time_values, self.state.kalman_velocity_values, label=kf_label)
        plt.ylabel('Velocity (m/s)')
        plt.legend()

        plt.subplot(4, 1, 3)
        plt.plot(self.state.time_values, [x[1] for x in self.state.acceleration_values], label='Acceleration')
        plt.plot(self.state.time_values, [x[1] for x in self.state.chute_acceleration_values], label='Chute Acceleration')
        plt.plot(self.state.time_values, self.state.kalman_acceleration_values, label=kf_label)
        plt.ylabel('Acceleration (m/s^2)')
        plt.legend()
        return plt


if __name__ == '__main__':
    sim = DescentSimulator()
    sim.simulate()
    sim.print_final_values()
    sim.plot('Descent Simulation Results').show()
