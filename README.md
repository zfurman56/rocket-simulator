# rocket-simulator
Rocket simulator created by the MA team for TARC 2017

## Note
This simulator allows for rapid testing and prototyping, but it does not contain actual flight code. For flight code, see <https://github.com/zfurman56/Firmware/tree/no-estimator>. For the simulation environment in which that code runs, see <https://github.com/zfurman56/jMAVSim>.

## Features
* Accurately models forces, velocity, and position in two dimensions
* Includes drag, gravity, and engine thrust
* Accepts engine thrust curves in the form of .eng files
* PID controller modulates simulated drag brakes
* Models sensor delays and inaccuracies, servo slew rate, and rocket launch tilt
* Graphs simulation data with matplotlib
* Includes post-flight analysis code that can create a function to estimate CdA given brake angle

## How to get started
* Clone the repo
* Install [NumPy](https://docs.scipy.org/doc/numpy-1.10.1/user/install.html) dependency
* Run the simulator with `python rocket_sim.py <thrust curve file>`
  * An example thrust curve, `AeroTech_F52.eng`, is included
* For drag brake curve-fitting (`analysis.py`), input collected CdA values and corresponding drag brake angles, then run `python analysis.py`.
