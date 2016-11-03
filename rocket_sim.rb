require 'matrix'

sim_complete = false

def get_rocket_commands
end

gravity = Vector[0, -9.8]

mass = 0.595 #kilograms
position = Vector[0, 100] #meters
rotation = 0 #degrees
velocity = Vector[0, 0] #meters/second

step_size = 0.01 #seconds
time = 0 #seconds

until sim_complete
  forces = Vector[0, 0]

  rocket_commands = get_rocket_commands

  forces += (gravity * mass)

  velocity += ((forces / mass) * step_size)
  position += (velocity * step_size)

  print position[1].to_s + ' ' + time.to_s + "\n"

  if position[1] < 0
    sim_complete = true
  end

  time += step_size
end
