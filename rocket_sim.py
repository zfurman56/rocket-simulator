import sys
import math
import numpy as np

def get_rocket_commands():
    pass

standard_deviation = 0.98 #percent
thrust_coefficient = (np.random.normal(100, standard_deviation) / 100)

# Supply .eng thrust file via command args
thrust_file = open(sys.argv[1], 'r')
thrust_lines = filter(lambda line: line[0]!=';', thrust_file.readlines())[1:-1]
raw_thrust = map(lambda line: map(lambda x: float(x)*thrust_coefficient, line.split('   ')[1:3]), thrust_lines)

# Raw thrust values plus interpolation
thrust = lambda x: np.interp(x, [item[0] for item in raw_thrust], [item[1] for item in raw_thrust], right=0)

gravity = np.array([0, -9.8])

mass = 0.595 #kilograms
position = np.array([0., 0.]) #meters
rotation = 0. #radians
velocity = np.array([0., 0.]) #meters/second

step_size = 0.01 #seconds
time = 0. #seconds

while True:

    forces = np.array([0., 0.])

    rocket_commands = get_rocket_commands

    forces += (gravity * mass)
    forces += (thrust(time) * np.array([math.sin(rotation), math.cos(rotation)]))
    forces += ((0.0008*np.linalg.norm(velocity)) * -velocity) #crappy drag model

    velocity += ((forces / mass) * step_size)
    position += (velocity * step_size)

    print str(position[1]) + ' ' + str(time)

    if position[1] < 0:
        break

    time += step_size

