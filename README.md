# rocket-simulator
2D Rocket simulator originally created by the MA team for TARC 2017, still in use as of 2018.

## Note
This simulator allows for rapid testing and prototyping, but it does not contain actual flight code. For flight code, see <https://github.com/zfurman56/Firmware/>. For the simulation environment in which that code runs, see <https://github.com/zfurman56/jMAVSim>.

## Features
* Accurately models forces, velocity, and position in two dimensions
* Includes drag, gravity, and engine thrust
* Accepts engine thrust curves in the form of .eng files
* PID controller modulates simulated drag brakes
* Kalman filter smooths data and fuses sensors
* Models sensor delays and inaccuracies, and servo slew rate
* Graphs simulation data with matplotlib
* Includes post-flight analysis code that can create a function to estimate CdA given brake angle
* Python2 and Python3 compatabile

## How to get started
* Clone the repo
* Install [NumPy, SciPy, and Matplotlib](https://scipy.org/install.html) and their dependencies.
  * (Specifically, on Linux, you may have to install the package `python-tkinter` for Matplotlib to work)
* Run the simulator with `python rocket_sim.py <thrust curve file>`
  * An example thrust curve, `AeroTech_F52.eng`, is included
* For drag brake curve-fitting (`analysis.py`), input collected CdA values and corresponding drag brake angles, then run `python analysis.py`.
