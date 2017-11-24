"""Simulates Apogee Control by use of drag brakes.

"""

import numpy as np
from matplotlib import pyplot as plt

from simulator import KFSimulator
from pid import PIDController
from utils import get_eng_file_from_argv
from params import (
    GRAVITY,
    BARO_STD,
    ACCEL_STD,
    MAX_SERVO_SLEW_RATE,
    MASS,
    TARGET_APOGEE,
    SIM_TIME_INC,
    CMD_PERIOD,
    KP, KI, KD,
    DRAG_FACTOR,
    DRAG_GAIN,
)


class ApogeeSimulator(KFSimulator):
    brake_angles = [0.] # Servo angle values for drag brakes (rads)

    def __init__(self, engine):
        super(ApogeeSimulator, self).__init__()
        self._eng = engine
        self.pid = PIDController(TARGET_APOGEE, KP, KI, KD)

    @property
    def brake_angle(self):
        return self.brake_angles[-1]

    @brake_angle.setter
    def brake_angle(self, angle):
        self.brake_angles.append(angle)

    def _get_drag_factor(self, brake_angle):
        """Map from drag brake angle to drag factor
        brake_angle: (rad)
        returns: velocity (m/s)
        """
        return DRAG_FACTOR * (1 + DRAG_GAIN * np.sin(brake_angle)**2)

    def _estimate_peak_altitude(self, alt, velocity, brake_angle):
        term_vel_sqrd = (MASS * -GRAVITY[1]) / self._get_drag_factor(brake_angle)
        return alt[1] + (term_vel_sqrd / (2 * -GRAVITY[1]) * np.log((velocity[1]**2 + term_vel_sqrd) / term_vel_sqrd))

    def _actuate(self, commanded_brake_angle):
        """Takes a slew rate for the drag brakes, clamps it,
        and uses it to compute the new brake angle.
        """
        commanded_brake_rate = commanded_brake_angle - self.brake_angle
        slew_rate = commanded_brake_rate / CMD_PERIOD
        clamped_slew_rate = np.clip(slew_rate, -MAX_SERVO_SLEW_RATE, MAX_SERVO_SLEW_RATE)
        return np.clip((self.brake_angle + (clamped_slew_rate * CMD_PERIOD)), 0, (np.pi/2))

    def _accelerometer_model(self, acceleration):
        """
        Accelerometer sensor model
        """
        accel_vector = np.cos(np.arctan2(acceleration[0], acceleration[1]) - self.pitch) * np.linalg.norm(acceleration)
        return np.random.normal(accel_vector, ACCEL_STD)

    def simulate(self):
        while not self.terminated:
            # Runs PID controller and returns commanded drag brake angle.
            # Actuate drag brakes if rocket is coasting
            est_apogee = self._estimate_peak_altitude(np.array([0, self.kf.x[0]]), np.array([0, self.kf.x[1]]), self.brake_angle)

            # Get the attempted servo command from the PID controller.
            # This may not end up being the actual servo angle, because
            # the servo can only move so fast
            brake_angle = self._actuate(self.brake_angle + self.pid.update(self.time, est_apogee))

            # TODO rewrite this portion; slight messy.
            sim_time_end = self.time + CMD_PERIOD - SIM_TIME_INC/2
            first = True
            while self.time < sim_time_end:
                self.brake_angle = brake_angle
                self.tick()

                self.kf.predict()
                if first:
                    # Barometer sensor inline
                    self.kf.update(np.array([np.random.normal(self.altitude[1], BARO_STD), self._accelerometer_model(self.acceleration - GRAVITY) + GRAVITY[1]]))
                else:
                    self.kf.update2(np.array([self._accelerometer_model(self.acceleration - GRAVITY) + GRAVITY[1]]))

                first = False

                # Appends values to lists
                self.estimated_altitude = self.kf.x[0]
                self.estimated_velocity = self.kf.x[1]
                self.estimated_acceleration = self.kf.x[2]

                # Increment the time
                self.time += SIM_TIME_INC

        self._print_results()

    def _calculate_forces(self):
        # Add Engine thrust and attitude to forces
        # Gravitational force
        forces = MASS * GRAVITY
        # Thrust and rotation
        forces += self._eng.thrust(self.time) * np.array([np.sin(self.pitch), np.cos(self.pitch)])
        # Air drag
        forces += self._get_drag_factor(self.brake_angle) * -self.velocity**2 * np.sign(self.velocity)
        return forces

    def plot(self, title=None):
        # create PID graph
        self.pid.figure('Apogee PID Controller Error')

        kf_label = 'KF estimation'

        plt.figure(title or 'Simulation Outcomes')
        plt.subplot(4, 1, 1)
        plt.plot(self.time_values, [x[1] for x in self.altitude_values], label='Altitude')
        plt.plot(self.time_values, self.kalman_altitude_values, label=kf_label)
        plt.ylabel('Altitude (m)')
        plt.legend()

        plt.subplot(4, 1, 2)
        plt.plot(self.time_values, [x[1] for x in self.velocity_values], label='Velocity')
        plt.plot(self.time_values, self.kalman_velocity_values, label=kf_label)
        plt.ylabel('Velocity (m/s)')
        plt.legend()

        plt.subplot(4, 1, 3)
        plt.plot(self.time_values, [x[1] for x in self.acceleration_values], label='Acceleration')
        plt.plot(self.time_values, self.kalman_acceleration_values, label=kf_label)
        plt.ylabel('Acceleration (m/s^2)')
        plt.legend()

        plt.subplot(4, 1, 4)
        plt.plot(self.time_values, [np.rad2deg(x) for x in self.brake_angles])
        plt.ylabel('Servo Angle (degrees)')
        plt.xlabel('Time (s)')

        return plt

if __name__ == '__main__':
    sim = ApogeeSimulator(get_eng_file_from_argv())
    sim.print_init_values()
    sim.simulate()
    sim.print_final_values()
    sim.plot('Apogee Simulation Results').show()
