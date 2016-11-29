# rocket-simulator
Rocket simulator created by the MA team for TARC 2017

## Features
* Accurately models forces, velocity, and position in two dimensions
* Includes drag, gravity, and engine thrust
* Accepts engine thrust curves in the form of .eng files
* Models engine thrust deviation with a Gaussian distribution
* PID controller modulates simulated drag brakes
* Models sensor delays and inaccuracies, servo slew rate, and launch rod tilt
* Graphs simulation data with matplotlib

## In the future
* Create a more accurate drag model
* Use actual flight data to improve simulation accuracy
* Currently only used for ascent; in the future, could also be used for descent
* Possible hardware-in-the-loop simulation

## How to get started
* Clone the repo
* Install [NumPy](https://docs.scipy.org/doc/numpy-1.10.1/user/install.html) dependency
* Run the program with `python rocket_sim.py <thrust curve file>`
  * An example thrust curve, `AeroTech_F52.eng`, is included
