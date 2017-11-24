"""
Drag brake curve-fitting tool

Fits drag brake angle vs CdA to a cubic function, allowing
drag estimation given brake angle.

"""

import numpy as np
from matplotlib import pyplot as plt


def to_rad(angle):
    return angle * (np.pi / 180)


def poly(coefs):
    return lambda x: ((coefs[0] * (x**3)) + (coefs[1] * (x**2)) + (coefs[2] * x) + coefs[3])


drag_brake_values = [0, 15, 30, 45, 60, 75, 90]
cda_values = [17.96, 26.38, 49.39, 80.82, 112.24, 135.25, 143.67]

rad_drag_brake_values = np.vectorize(to_rad)(drag_brake_values)
m2_cda_values = np.divide(cda_values, 10000)

polycoefs = np.polyfit(rad_drag_brake_values, m2_cda_values, deg=3)
print('Polynomial coefficients:', polycoefs)
polyfunc = poly(polycoefs)

polyfit_drag_brake_values = np.linspace(0, (np.pi/2), 90)
polyfit_cda_values = np.vectorize(polyfunc)(polyfit_drag_brake_values)


plt.plot(rad_drag_brake_values, m2_cda_values, 'co')
plt.plot(polyfit_drag_brake_values, polyfit_cda_values)
plt.ylabel('CdA (m^2)')
plt.xlabel('Drag Brake Angle (radians)')
plt.show()
