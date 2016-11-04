require 'matrix'
require 'interpolator'
require 'distribution'

sim_complete = false

def get_rocket_commands
end

standard_deviation = 0.98 #percent
thrust_coefficient = (Distribution::Normal.rng(100, standard_deviation).call / 100)

# Supply .eng thrust file via command args
thrust_file = File.open(ARGV[0], 'r')
thrust_lines = thrust_file.each_line.reject{|line| line[0]==';' }[1..-1]
raw_thrust = thrust_lines.map{|line| line.chomp.split('   ')[1..2].map{|n|n.to_f*thrust_coefficient}}

# Raw thrust values plus interpolation
thrust = Interpolator::Table.new raw_thrust.map(&:first), raw_thrust.map(&:last)
thrust.extrapolate = false

gravity = Vector[0, -9.8]

mass = 0.595 #kilograms
position = Vector[0, 0] #meters
rotation = 0 #radians
velocity = Vector[0, 0] #meters/second

step_size = 0.01 #seconds
time = 0 #seconds

until sim_complete
  forces = Vector[0, 0]

  rocket_commands = get_rocket_commands

  forces += (gravity * mass)
  forces += (thrust.interpolate(time) * Vector[Math::sin(rotation), Math::cos(rotation)])
  forces += ((0.0008*velocity.r) * -velocity) #crappy drag model

  velocity += ((forces / mass) * step_size)
  position += (velocity * step_size)

  print position[1].to_s + ' ' + time.to_s + "\n"

  if position[1] < 0
    sim_complete = true
  end

  time += step_size
end
