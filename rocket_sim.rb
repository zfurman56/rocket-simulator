require 'matrix'

sim_complete = false

def get_rocket_commands
end

position = Vector[0, 0] #meters
rotation = 0 #degrees

step_size = 0.01 #seconds
time = 0 #seconds

until sim_complete
  rocket_commands = get_rocket_commands

  time += step_size
end