import sys
import math
import numpy as np
import matplotlib.pyplot as plt

def get_rocket_commands(position, velocity):
    if velocity[1]*-0.372 < position[1]-242.2:
        return True
    else:
        return False

baro_std = 0.3
drag_factor = 0.0008
drag_gain = 100
thrust_scale = 1.02

# Supply .eng thrust file via command args
thrust_file = open(sys.argv[1], 'r')
thrust_lines = filter(lambda line: line[0]!=';', thrust_file.readlines())[1:-1]
raw_thrust = map(lambda line: map(lambda x: float(x)*thrust_scale, line.split('   ')[1:3]), thrust_lines)

raw_times = [item[0] for item in raw_thrust]
raw_thrusts = [item[1] for item in raw_thrust]

# Raw thrust values plus interpolation
thrust = lambda x: np.interp(x, raw_times, raw_thrusts, right=0)

gravity = np.array([0, -9.8])

mass = 0.595 #kilograms
position = np.array([0., 0.]) #meters
rotation = 0. #radians
velocity = np.array([0., 0.]) #meters/second
est_position = np.array([0., 0.])

step_size = 0.01 #seconds
time = 0. #seconds

# Used for delay
rocket_commands = [False] * 10

# Models barometric sensor inaccuracy
def sensor_model(position, previous_est_position):
    est_position = np.array([0., np.random.normal(position[1], baro_std)])
    est_velocity = (position - previous_est_position) / step_size
    return est_position, est_velocity

# Generates a coeffecient to be multiplied with velocity squared
def drag_coeffecient(chute_deployed):
    return drag_factor * (1 + drag_gain) if chute_deployed else drag_factor

# Computes one simulated time step
def sim_step(time, position, velocity, rotation, chute_deployed):
    forces = (gravity * mass)
    forces += (thrust(time) * np.array([math.sin(rotation), math.cos(rotation)]))
    forces += drag_coeffecient(chute_deployed) * -(velocity*np.abs(velocity))

    velocity += ((forces / mass) * step_size)
    position += (velocity * step_size)

    time += step_size

    return (time, position, velocity, rotation)

# Used for data plotting
altitude_values = []
time_values = []

while True:

    previous_est_position = np.copy(est_position)
    est_position, est_velocity = sensor_model(position, previous_est_position)

    forces = np.array([0., 0.])

    rocket_commands.append(get_rocket_commands(est_position, est_velocity))

    new_time, position, velocity, rotation = sim_step(time, position, velocity, rotation, rocket_commands.pop(0))

    print str(position[1]) + ' ' + str(time)
    altitude_values.append(position[1])
    time_values.append(time)

    if position[1] < 0:
        break

    time = new_time

print "Peak altitude: " + str(max(altitude_values))

plt.plot(time_values, altitude_values)
plt.ylabel('Rocket altitude vs time')
plt.show()
