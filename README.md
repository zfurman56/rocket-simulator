# rocket-simulator
Rocket simulator created by the MA team for TARC 2017

## Features
* Accurately models forces, velocity, and position in two dimensions
* Includes drag, gravity, and engine thrust
* Accepts engine thrust curves in the form of .eng files
* Models engine thrust deviation with a Gaussian distribution

## In the future
* Create a PID controller
  * Later, move the PID to flight computer
* Test several methods of control and compare viability
* Model several stages of flight (ascent, descent, etc.)
  * Create control methods for each stage
* Add moments and other rotational features
* Make the system 3D (if needed)
* Create a more accurate drag model
* Use actual flight data to improve simulation accuracy
* Possibly hardware-in-the-loop simulation

## How to get started
* Clone the repo
* Install numpy dependency
* Run the program with `python rocket_sim.py <thrust curve file>`
  * An example thrust curve, `AeroTech_F39.eng`, is included
