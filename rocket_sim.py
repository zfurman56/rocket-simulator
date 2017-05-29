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

gravity = np.array([0, -9.8])

mass = 0.550 #kilograms
dry_mass = 0.270 #kilograms
step_size = 0.01 #seconds
drag_coefficient = 0.18
frontal_area = 0.0095 #m^2
air_density = 1.225 #kg/m^3
thrust_scale = 1

# Supply .eng thrust file via command args
thrust_file = open(sys.argv[1], 'r')
thrust_lines = filter(lambda line: line[0]!=';', thrust_file.readlines())[1:-1]
raw_thrust = map(lambda line: map(lambda x: float(x), line.split(' ')), thrust_lines)

raw_times = [item[0] for item in raw_thrust]
raw_thrusts = [item[1]*thrust_scale for item in raw_thrust]

# Raw thrust values plus interpolation
thrust = lambda x: np.interp(x, raw_times, raw_thrusts, right=0)

# For input to plotting
time_values = []
altitude_values = []
velocity_values = []

# Calculate drag force
def drag_force(velocity):
    return 0.5 * drag_coefficient * frontal_area * air_density * np.sign(velocity[1]) * -(velocity ** 2)

# Computes one simulated time step
def sim_step(time, position, velocity, rotation):
    forces = (gravity * mass)
    forces += thrust(time) * np.array([math.sin(rotation), math.cos(rotation)])
    forces += drag_force(velocity)

    new_velocity = velocity + ((forces / mass) * step_size)
    new_position = position + (new_velocity * step_size)

    time += step_size

    return (time, new_position, new_velocity, rotation)

# Main simulation loop
def sim():

    global altitude_values, velocity_values, time_values

    # State vars
    time = 0. #seconds
    position = np.array([0., 0.]) #meters
    rotation = 0.0 #radians
    velocity = np.array([0., 0.]) #meters/second

    while True:

        time, position, velocity, rotation = sim_step(time, position, velocity, rotation)

        print str(position[1]) + ' ' + str(time)
        altitude_values.append(position[1])
        velocity_values.append(velocity[1])
        time_values.append(time)

        if position[1] < 0:
            break

sim()

print "\nPeak altitude: " + str(max(altitude_values)) + " meters"
print "Max velocity: " + str(max(velocity_values)) + " m/s"
print "Impact velocity: " + str(velocity_values[-1]) + " m/s"
print "Impact energy: " + str((0.5 * dry_mass * (velocity_values[-1]**2))) + " Joules"
print "Impulse to stop egg: " + str(-velocity_values[-1]*0.05) + " kg*m/s"
print "Flight time: " + str(time_values[-1]) + " seconds\n"

plt.subplot(2,1,1)
plt.plot(time_values, altitude_values)
plt.ylabel('Altitude (m)')
plt.xlabel('Time (s)')

plt.subplot(2,1,2)
plt.plot(time_values, velocity_values)
plt.ylabel('Velocity (m)')
plt.xlabel('Time (s)')

plt.show()
