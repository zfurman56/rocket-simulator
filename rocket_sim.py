###
# Core simulation code
#
# Full 2-D simulation of rocket, including engine thrust,
# drag, and gravity. Includes drag brakes controlled by a
# closed-loop PID controller.
###

import sys
import math
import numpy as np
import matplotlib.pyplot as plt
from my_kalman import kfilter

gravity = np.array([0, -9.8])

baro_std = 0.3  # Baro altitude sensor stddev (m)
accel_std = 0.1  # Accelerometer sensor stddev (m/s^2)
max_servo_slew_rate = math.pi * 2 # rad/s
mass = 0.625 #kilograms
target_altitude = 236 # meters
rod_length = 0
step_size = 0.00416666666 #seconds
cmd_period = 0.05 #seconds
kp = 0.008
ki = 0.0
kd = 0.0
drag_factor = 0.00185
es_drag_factor = 0.00185
drag_gain = 10
thrust_scale = 1.0

# Supply .eng thrust file via command args
thrust_file = open(sys.argv[1], 'r')
thrust_lines = filter(lambda line: line[0]!=';', thrust_file.readlines())[1:-1]
raw_thrust = map(lambda line: map(lambda x: float(x), line.split(' ')), thrust_lines)

raw_times = [item[0] for item in raw_thrust]
raw_thrusts = [item[1]*thrust_scale for item in raw_thrust]

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

# Map from drag brake angle to drag force
#   drag_brake_angle (rad)
#   velocity (m/s)
def get_drag_factor(drag_brake_angle, is_estimation):
    if is_estimation:
        return es_drag_factor * (1 + (drag_gain * (math.sin(drag_brake_angle)**2)))
    else:
        return drag_factor * (1 + (drag_gain * (np.sin(drag_brake_angle)**2)))

# Takes a slew rate for the drag brakes, clamps it, and uses it to compute the new brake angle
def actuate(commanded_brake_rate, current_angle):
    slew_rate = commanded_brake_rate / cmd_period
    clamped_slew_rate = np.clip(slew_rate, -max_servo_slew_rate, max_servo_slew_rate)
    new_angle = np.clip((current_angle + (clamped_slew_rate * cmd_period)), 0, (math.pi/2))
    servo_angle_values.append((new_angle*(180/math.pi)))
    return new_angle

# Computes one simulated time step
def sim_step(time, position, velocity, rotation, drag_brake_angle, is_estimation):
    forces = (gravity * mass)
    forces += thrust(time) * np.array([math.sin(rotation), math.cos(rotation)])
    forces += get_drag_factor(drag_brake_angle, is_estimation) * -(velocity ** 2) * np.sign(velocity)

    acceleration = (forces / mass)
    new_velocity = velocity + (acceleration * step_size)
    new_position = position + (velocity * step_size) + (0.5 * acceleration * (step_size**2))

    time += step_size

    if np.linalg.norm(position) > rod_length:
        rotation = math.pi/2 - math.atan2(new_velocity[1], new_velocity[0])

    return (time, new_position, new_velocity, rotation, acceleration)

# Estimates apogee altitude with given parameters
def estimate_peak_altitude(position, velocity, drag_brake_angle):
    term_vel_sqrd = (mass * -gravity[1]) / get_drag_factor(drag_brake_angle, True)
    return position[1] + ((term_vel_sqrd / (2 * -gravity[1])) * np.log((((velocity[1] ** 2) + term_vel_sqrd) / term_vel_sqrd)))

# Get standard deviation of pitot tube velocity given pressure accuracy in Pascals and current velocity
# Uses equations from https://en.wikipedia.org/wiki/Pitot_tube
def get_pitot_accuracy(pressure_accuracy, velocity):
    # Differential pressure (difference between static and total)
    pressure = (1.225 * velocity[1]**2)/2

    if pressure > pressure_accuracy:
        return 0.5 * (math.sqrt((2 * (pressure + pressure_accuracy)) / 1.225) -
                      math.sqrt((2 * (pressure - pressure_accuracy)) / 1.225))
    else:
        return 0.5 * (math.sqrt((2 * (pressure + pressure_accuracy)) / 1.225) +
                      math.sqrt((-2 * (pressure - pressure_accuracy)) / 1.225))

# Used as buffer, to simulate delay in baro and GPSs
est_positions = []
# est_velocities = [0]*2
est_accels = []

# # Models barometric sensor inaccuracy
# def sensor_model(position, acceleration, rotation):
#     est_positions.append(np.random.normal(position[1], baro_std))
#     pitot_velocity = np.random.normal(velocity, get_pitot_accuracy((2*68.9), velocity))
#     gps_velocity = np.random.normal(velocity, gps_std)
#     est_velocities.append(pitot_velocity)
#     est_accels.append(np.random.normal(acceleration[1], accel_std))
#     return est_positions.pop(0), est_accels.pop(0)

def baro_model(position):
    est_positions.append(np.random.normal(position[1], baro_std))
    return est_positions.pop(0)

def accel_model(acceleration, rotation):
    # acceleration vector projected to rotation
    accel_vector = np.cos(np.arctan2(acceleration[0], acceleration[1])-rotation) * np.linalg.norm(acceleration)
    est_accels.append(np.random.normal(accel_vector, accel_std))
    return est_accels.pop(0)

# Runs PID controller and returns commanded drag brake angle
def get_rocket_command(time, est_position, est_velocity, rotation, drag_brake_angle):
    global previous_error
    global integrated_error

    #print "est_position: " + str(est_position) + "  est_velocity: " + str(est_velocity)

    error = estimate_peak_altitude(est_position, est_velocity, drag_brake_angle) - target_altitude

    error_values.append(error)
    time_values.append(time)

    integrated_error += error
    derivative_error = (error - previous_error)/cmd_period

    previous_error = error

    new_drag_brake_rate = kp*error + ki*integrated_error + kd*derivative_error

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
        rate_cmd = get_rocket_command(time, np.array([0, kfilter.x[0]]), np.array([0, kfilter.x[1]]), rotation, servo_angle)
        servo_angle = actuate(rate_cmd, servo_angle)

        sim_time_end = time + cmd_period - step_size/2
        first = True
        while time < sim_time_end:
            if time > 0:
                time, position, velocity, rotation, acceleration = sim_step(time, position, velocity, rotation, servo_angle, False)
            else:
                time += step_size

            kfilter.predict()
            if first:
                kfilter.update(np.array([baro_model(position), accel_model(acceleration-gravity, rotation)+gravity[1]]))
            else:
                kfilter.update2(np.array([accel_model(acceleration-gravity, rotation)+gravity[1]]))

            first = False

            print str(position[1]) + ' ' + str(time)
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

print "Peak altitude: " + str(max(altitude_values))
print "Flight time: " + str(max(altitude_time_values))

plt.subplot(5,1,1)
plt.plot(altitude_time_values, altitude_values)
plt.plot(altitude_time_values, kalman_altitude_values)
plt.ylabel('Altitude (m)')
plt.xlabel('Time (s)')

plt.subplot(5,1,2)
plt.plot(altitude_time_values, velocity_values)
plt.plot(altitude_time_values, kalman_velocity_values)
plt.ylabel('Velocity (m/s)')
plt.xlabel('Time (s)')

plt.subplot(5,1,3)
plt.plot(altitude_time_values, accel_values)
plt.plot(altitude_time_values, kalman_accel_values)
plt.ylabel('Acceleration (m/s^2)')
plt.xlabel('Time (s)')

plt.subplot(5,1,4)
plt.plot(time_values, error_values)
plt.ylabel('PID error (m)')
plt.xlabel('Time (s)')

plt.subplot(5,1,5)
plt.plot(time_values, servo_angle_values)
plt.ylabel('Servo Angle (degrees)')
plt.xlabel('Time (s)')

plt.show()
