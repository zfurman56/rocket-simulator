"""
Core simulation code

Full 2-D simulation of rocket, including engine thrust,
drag, and gravity. Includes drag brakes controlled by a
closed-loop PID controller.
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
from my_kalman import kfilter

import compat
from engparser import NARThrustParser
from pid import PIDController
from utils import eng_filename_argv
from const import (
    GRAVITY,
    BARO_STD,
    ACCEL_STD,
    MAX_SERVO_SLEW_RATE,
    MASS,
    TARGET_APOGEE,
    ROD_LEN,
    STEP_SIZE,
    CMD_PERIOD,
    KP, KI, KD,
    DRAG_FACTOR,
    ES_DRAG_FACTOR,
    DRAG_GAIN
)

# Supply .eng thrust file via command args
with open(eng_filename_argv(sys.argv)) as f:
    ntp = NARThrustParser(f)
    # Raw thrust values plus interpolation
    thrust = lambda x: np.interp(x, ntp.time, ntp.thrust, right=0)


# Global vars
previous_error = 0
integrated_error = 0

# For input to plotting
error_values = []
time_values = []
servo_angle_values = []
altitude_time_values = []
kalman_altitude_values = []
kalman_velocity_values = []
kalman_accel_values = []
altitude_values = []
velocity_values = []
accel_values = []

def get_drag_factor(drag_brake_angle, is_estimation):
    """Map from drag brake angle to drag force
    drag_brake_angle: (rad)
    returns: velocity (m/s)
    """
    vel = 1 + DRAG_GAIN * np.sin(drag_brake_angle)**2
    if is_estimation:
        return ES_DRAG_FACTOR * vel
    return DRAG_FACTOR * vel

def actuate(commanded_brake_angle, current_angle):
    """Takes a slew rate for the drag brakes, clamps it,
    and uses it to compute the new brake angle.
    """
    commanded_brake_rate = commanded_brake_angle-current_angle
    slew_rate = commanded_brake_rate / CMD_PERIOD
    clamped_slew_rate = np.clip(slew_rate, -MAX_SERVO_SLEW_RATE, MAX_SERVO_SLEW_RATE)
    new_angle = np.clip((current_angle + (clamped_slew_rate * CMD_PERIOD)), 0, (np.pi/2))
    servo_angle_values.append((new_angle*(180/np.pi)))
    return new_angle

def sim_step(time, position, velocity, rotation, drag_brake_angle, is_estimation):
    """Computes one simulated time step.
    """
    forces = GRAVITY * MASS
    forces += thrust(time) * np.array([np.sin(rotation), np.cos(rotation)])
    forces += get_drag_factor(drag_brake_angle, is_estimation) * -(velocity**2) * np.sign(velocity)

    acceleration = (forces / MASS)
    new_velocity = velocity + (acceleration * STEP_SIZE)
    new_position = position + (velocity * STEP_SIZE) + (0.5 * acceleration * STEP_SIZE**2)

    time += STEP_SIZE

    if np.linalg.norm(position) > ROD_LEN:
        rotation = np.pi/2 - np.arctan2(new_velocity[1], new_velocity[0])

    return (time, new_position, new_velocity, rotation, acceleration)

def estimate_peak_altitude(position, velocity, drag_brake_angle):
    """Estimates apogee altitude with given parameters.
    """
    term_vel_sqrd = (MASS * -GRAVITY[1]) / get_drag_factor(drag_brake_angle, True)
    return position[1] + (term_vel_sqrd / (2 * -GRAVITY[1]) *
                          np.log((velocity[1]**2 + term_vel_sqrd) / term_vel_sqrd))

def get_pitot_accuracy(pressure_accuracy, velocity):
    """Get standard deviation of pitot tube velocity given pressure
    accuracy in Pascals and current velocity.

    Uses equations from https://en.wikipedia.org/wiki/Pitot_tube
    """
    # Differential pressure (difference between static and total)
    pressure = (1.225 * velocity[1]**2)/2

    if pressure > pressure_accuracy:
        return 0.5 * (np.sqrt((2 * (pressure + pressure_accuracy)) / 1.225) -
                      np.sqrt((2 * (pressure - pressure_accuracy)) / 1.225))
    else:
        return 0.5 * (np.sqrt((2 * (pressure + pressure_accuracy)) / 1.225) +
                      np.sqrt((-2 * (pressure - pressure_accuracy)) / 1.225))

# Used as buffer, to simulate delay in baro and GPSs
est_positions = []
# est_velocities = [0]*2
est_accels = []

# def sensor_model(position, acceleration, rotation):
#     """Models barometric sensor inaccuracy.
#     """
#     est_positions.append(np.random.normal(position[1], BARO_STD))
#     pitot_velocity = np.random.normal(velocity, get_pitot_accuracy((2*68.9), velocity))
#     gps_velocity = np.random.normal(velocity, gps_std)
#     est_velocities.append(pitot_velocity)
#     est_accels.append(np.random.normal(acceleration[1], accel_std))
#     return est_positions.pop(0), est_accels.pop(0)

def baro_model(position):
    est_positions.append(np.random.normal(position[1], BARO_STD))
    return est_positions.pop(0)

def accel_model(acceleration, rotation):
    """"Acceleration vector projected to rotation.
    """
    accel_vector = np.cos(np.arctan2(acceleration[0],
                                     acceleration[1])-rotation) * np.linalg.norm(acceleration)
    est_accels.append(np.random.normal(accel_vector, ACCEL_STD))
    return est_accels.pop(0)

# Main simulation loop
def sim():

    global altitude_values, velocity_values, altitude_time_values

    # State vars
    time = -5. #seconds
    servo_angle = 0  # Brake angle (rad)
    position = np.array([0., 0.]) #meters
    rotation = 0.0 #radians
    velocity = np.array([0., 0.]) #meters/second
    acceleration = np.array([0., 0.])
    # est_position = 0

    pid = PIDController(TARGET_APOGEE, KP, KI, KD)

    while True:
        # previous_est_position = np.copy(est_position)

        # Runs PID controller and returns commanded drag brake angle.
        # Actuate drag brakes if rocket is coasting
        est_apogee = estimate_peak_altitude(np.array([0, kfilter.x[0]]),
                                       np.array([0, kfilter.x[1]]), servo_angle)
        time_values.append(time)

        # Get the attempted servo command from the PID controller.
        # This may not end up being the actual servo angle, because
        # the servo can only move so fast
        desired_angle = pid.update(time, servo_angle, est_apogee)
        servo_angle = actuate(desired_angle, servo_angle)

        sim_time_end = time + CMD_PERIOD - STEP_SIZE/2
        first = True
        while time < sim_time_end:
            if time > 0:
                time, position, velocity, rotation, acceleration = sim_step(time, position,
                                                                            velocity, rotation,
                                                                            servo_angle, False)
            else:
                time += STEP_SIZE

            kfilter.predict()
            if first:
                kfilter.update(np.array([baro_model(position),
                                         accel_model(acceleration-GRAVITY, rotation)+GRAVITY[1]]))
            else:
                kfilter.update2(np.array([accel_model(acceleration-GRAVITY, rotation)+GRAVITY[1]]))

            first = False

            # print('Position:', position[1], 'Time (sec):', time)
            altitude_values.append(position[1])
            kalman_altitude_values.append(kfilter.x[0])
            kalman_velocity_values.append(kfilter.x[1])
            kalman_accel_values.append(kfilter.x[2])
            accel_values.append(acceleration[1])
            velocity_values.append(velocity[1])
            altitude_time_values.append(time)

        if position[1] < 0:
            break
    pid.figure()

sim()

print('Peak altitude (m):', max(altitude_values))
print('Flight time (sec):', max(altitude_time_values))

truth_label = 'Truth'
kf_label = 'KF estimation'

plt.figure('Simulation Outcomes')
plt.subplot(4, 1, 1)
plt.plot(altitude_time_values, altitude_values, label=truth_label)
plt.plot(altitude_time_values, kalman_altitude_values, label=kf_label)
plt.ylabel('Altitude (m)')
plt.xlabel('Time (s)')
plt.legend()

plt.subplot(4, 1, 2)
plt.plot(altitude_time_values, velocity_values, label=truth_label)
plt.plot(altitude_time_values, kalman_velocity_values, label=kf_label)
plt.ylabel('Velocity (m/s)')
plt.xlabel('Time (s)')
plt.legend()

plt.subplot(4, 1, 3)
plt.plot(altitude_time_values, accel_values, label=truth_label)
plt.plot(altitude_time_values, kalman_accel_values, label=kf_label)
plt.ylabel('Acceleration (m/s^2)')
plt.xlabel('Time (s)')
plt.legend()

plt.subplot(4, 1, 4)
plt.plot(time_values, servo_angle_values)
plt.ylabel('Servo Angle (degrees)')
plt.xlabel('Time (s)')

plt.show()
