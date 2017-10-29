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
    DRAG_GAIN,
    THRUST_SCALE
)

# Supply .eng thrust file via command args
thrust_file = open(sys.argv[1], 'r')
thrust_lines = [x for x in thrust_file.readlines() if x[0]!=';'][1:-1]
raw_thrust = [[float(x) for x in line.split(' ')] for line in thrust_lines]

raw_times = [item[0] for item in raw_thrust]
raw_thrusts = [item[1]*THRUST_SCALE for item in raw_thrust]

# Raw thrust values plus interpolation
thrust = lambda x: np.interp(x, raw_times, raw_thrusts, right=0)


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

def actuate(commanded_brake_rate, current_angle):
    """Takes a slew rate for the drag brakes, clamps it,
    and uses it to compute the new brake angle.
    """
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

def get_rocket_command(time, est_position, est_velocity, rotation, drag_brake_angle):
    """Runs PID controller and returns commanded drag brake angle.
    """
    global previous_error
    global integrated_error

    # print('est_position:', est_position, 'est_velocity (m/s):', est_velocity)

    error = estimate_peak_altitude(est_position, est_velocity, drag_brake_angle) - TARGET_APOGEE

    error_values.append(error)
    time_values.append(time)

    integrated_error += error
    derivative_error = (error - previous_error)/CMD_PERIOD

    previous_error = error

    new_drag_brake_rate = KP*error + KI*integrated_error + KD*derivative_error

    return new_drag_brake_rate


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

    while True:

        # previous_est_position = np.copy(est_position)

        # Actuate drag brakes if rocket is coasting
        rate_cmd = get_rocket_command(time, np.array([0, kfilter.x[0]]),
                                      np.array([0, kfilter.x[1]]), rotation, servo_angle)
        servo_angle = actuate(rate_cmd, servo_angle)

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

            print('Position:', position[1], 'Time (sec):', time)
            altitude_values.append(position[1])
            kalman_altitude_values.append(kfilter.x[0])
            kalman_velocity_values.append(kfilter.x[1])
            kalman_accel_values.append(kfilter.x[2])
            accel_values.append(acceleration[1])
            velocity_values.append(velocity[1])
            altitude_time_values.append(time)

        if position[1] < 0:
            break

sim()

print('Peak altitude (m):', max(altitude_values))
print('Flight time (sec):', max(altitude_time_values))

plt.subplot(5, 1, 1)
plt.plot(altitude_time_values, altitude_values)
plt.plot(altitude_time_values, kalman_altitude_values)
plt.ylabel('Altitude (m)')
plt.xlabel('Time (s)')

plt.subplot(5, 1, 2)
plt.plot(altitude_time_values, velocity_values)
plt.plot(altitude_time_values, kalman_velocity_values)
plt.ylabel('Velocity (m/s)')
plt.xlabel('Time (s)')

plt.subplot(5, 1, 3)
plt.plot(altitude_time_values, accel_values)
plt.plot(altitude_time_values, kalman_accel_values)
plt.ylabel('Acceleration (m/s^2)')
plt.xlabel('Time (s)')

plt.subplot(5, 1, 4)
plt.plot(time_values, error_values)
plt.ylabel('PID error (m)')
plt.xlabel('Time (s)')

plt.subplot(5, 1, 5)
plt.plot(time_values, servo_angle_values)
plt.ylabel('Servo Angle (degrees)')
plt.xlabel('Time (s)')

plt.show()
