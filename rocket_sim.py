import sys
import math
import numpy as np
import matplotlib.pyplot as plt

gravity = np.array([0, -9.8])

baro_std = 0.3  # Baro altitude sensor stddev (m)
max_servo_slew_rate = math.pi / 2 # rad/s
mass = 0.595 #kilograms
target_altitude = 236 # meters
step_size = 0.01 #seconds
cmd_period = 0.05 #seconds
kp = 0.05
ki = 0.0
kd = 0.0
drag_factor = 0.0008
drag_gain = 2
thrust_scale = 1.02

# Supply .eng thrust file via command args
thrust_file = open(sys.argv[1], 'r')
thrust_lines = filter(lambda line: line[0]!=';', thrust_file.readlines())[1:-1]
raw_thrust = map(lambda line: map(lambda x: float(x)*thrust_scale, line.split('   ')[1:3]), thrust_lines)

raw_times = [item[0] for item in raw_thrust]
raw_thrusts = [item[1] for item in raw_thrust]

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
altitude_values = []

# Map from drag brake angle to drag force
#   drag_brake_angle (rad)
#   velocity (m/s)
def drag_force(drag_brake_angle, velocity):
    return drag_factor * (1 + drag_gain * math.sin(drag_brake_angle)) * -(velocity ** 2)

# Takes a slew rate for the drag brakes, clamps it, and uses it to compute the new brake angle
def actuate(commanded_brake_rate, current_angle):
    slew_rate = commanded_brake_rate / step_size
    clamped_slew_rate = np.clip(slew_rate, -max_servo_slew_rate, max_servo_slew_rate)
    new_angle = current_angle + (clamped_slew_rate * step_size)
    servo_angle_values.append(new_angle)
    return new_angle

# Computes one simulated time step
def sim_step(time, position, velocity, rotation, drag_brake_angle):
    forces = (gravity * mass)
    forces += thrust(time) * np.array([math.sin(rotation), math.cos(rotation)])
    forces += drag_force(drag_brake_angle, velocity)

    new_velocity = velocity + ((forces / mass) * step_size)
    new_position = position + (velocity * step_size)

    time += step_size

    return (time, new_position, new_velocity, rotation)

# Estimates apogee altitude with given parameters
def estimate_peak_altitude(time, position, velocity, rotation, drag_brake_angle):
    while True:
        time, position, velocity, rotation = sim_step(time, position, velocity, rotation, drag_brake_angle)
        if velocity[1] < 0:
            return position[1]

# Models barometric sensor inaccuracy
def sensor_model(position, previous_est_position):
    est_position = np.array([0., np.random.normal(position[1], baro_std)])
    est_velocity = (position - previous_est_position) / cmd_period
    return est_position, est_velocity

# Runs PID controller and returns commanded drag brake angle
def get_rocket_command(time, est_position, est_velocity, rotation, drag_brake_angle):
    global previous_error
    global integrated_error

    #print "est_position: " + str(est_position) + "  est_velocity: " + str(est_velocity)

    error = estimate_peak_altitude(time, est_position, est_velocity, rotation, drag_brake_angle) - target_altitude

    error_values.append(error)
    time_values.append(time)

    integrated_error += error
    derivative_error = (error - previous_error)/cmd_period

    previous_error = error

    new_drag_brake_rate = kp*error + ki*integrated_error + kd*derivative_error

    return new_drag_brake_rate


# Main simulation loop
def sim():

    global altitude_values, altitude_time_values

    # State vars
    time = 0. #seconds
    servo_angle = 0  # Brake angle (rad)
    position = np.array([0., 0.]) #meters
    rotation = 0. #radians
    velocity = np.array([0., 0.]) #meters/second
    est_position = 0

    while True:

        previous_est_position = np.copy(est_position)
        est_position, est_velocity = sensor_model(position, previous_est_position)

        if ((thrust(time) == 0) and (velocity[1] > 0)):
            rate_cmd = get_rocket_command(time, est_position, est_velocity, rotation, servo_angle)
            servo_angle = actuate(rate_cmd, servo_angle)

        sim_time_end = time + cmd_period - step_size/2
        while time < sim_time_end:
            time, position, velocity, rotation = sim_step(time, position, velocity, rotation, servo_angle)

            print str(position[1]) + ' ' + str(time)
            altitude_values.append(position[1])
            altitude_time_values.append(time)

        if position[1] < 0:
            break

sim()

print "Peak altitude: " + str(max(altitude_values))

plt.plot(altitude_time_values, altitude_values)
plt.ylabel('Altitude (m)')
plt.xlabel('Time (s)')

plt.figure()
plt.plot(time_values, error_values)
plt.ylabel('PID error (m)')
plt.xlabel('Time (s)')

plt.figure()
plt.plot(time_values, servo_angle_values)
plt.ylabel('Servo Angle (rad)')
plt.xlabel('Time (s)')

plt.show()
