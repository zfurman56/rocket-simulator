"""Simulates Apogee Control by use of drag brakes.

"""
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
)


class DescentSimulator(Simulator):
    class DescentState(KFState):
        def __init__(self, apogee=None):
            '''Assumes velocity and acceleration are
            0m/s and 9.8m/s^2 (gravity) respectively.
            '''
            self.chute_deployed = False
            # Phys
            super(DescentSimulator.DescentState, self).__init__()
            self.altitude_values = [apogee or np.array([0., TARGET_APOGEE])] # m
            self.acceleration_values = [GRAVITY] # m/s^2
            self.time_values = [6.3] # seconds

            # kalman filter
            self.kf.x = [244., # altitude - meters
                         0., # velocity - m/s
                         GRAVITY[1]] # acceleration - m/s^2

    def __init__(self, state=None):
        super(DescentSimulator, self).__init__(state or DescentSimulator.DescentState())
        assert isinstance(self.state, KFState), 'State must be of type `KFState` or subclass for descent control simulation.'
        self.pid = PIDController(TARGET_APOGEE, KP, KI, KD)


    # def _actuate(self, commanded_brake_angle):
    #     commanded_brake_rate = commanded_brake_angle - self.state.brake_angle
    #     slew_rate = commanded_brake_rate / CMD_PERIOD
    #     clamped_slew_rate = np.clip(slew_rate, -MAX_SERVO_SLEW_RATE, MAX_SERVO_SLEW_RATE)
    #     return np.clip((self.state.brake_angle + (clamped_slew_rate * CMD_PERIOD)), 0, (np.pi/2))

    def simulate(self):
        while not self.terminated:
            # Deploy chute after 3 seconds (example)
            if self.state.time > self.state.time_values[0] + 3: self.state.chute_deployed = True

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

    def _calculate_forces(self):
        # Assumes drag brakes are not in use
        if not self.state.chute_deployed:
            return super(DescentSimulator, self)._calculate_forces()
        forces = MASS * GRAVITY
        # forces += CHUTE_DRAG_FACTOR * (1 + DRAG_GAIN) * -self.state.velocity**2 * np.sign(self.state.velocity)
        forces += CHUTE_DRAG_FACTOR * -self.state.velocity**2 * np.sign(self.state.velocity)
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
        plt.plot(self.state.time_values, self.state.kalman_altitude_values, label=kf_label)
        plt.ylabel('Altitude (m)')
        plt.legend()

        plt.subplot(4, 1, 2)
        plt.plot(self.state.time_values, [x[1] for x in self.state.velocity_values], label='Velocity')
        plt.plot(self.state.time_values, self.state.kalman_velocity_values, label=kf_label)
        plt.ylabel('Velocity (m/s)')
        plt.legend()

        plt.subplot(4, 1, 3)
        plt.plot(self.state.time_values, [x[1] for x in self.state.acceleration_values], label='Acceleration')
        plt.plot(self.state.time_values, self.state.kalman_acceleration_values, label=kf_label)
        plt.ylabel('Acceleration (m/s^2)')
        plt.legend()
        return plt


if __name__ == '__main__':
    sim = DescentSimulator()
    sim.simulate()
    sim.print_final_values()
    sim.plot('Descent Simulation Results').show()
