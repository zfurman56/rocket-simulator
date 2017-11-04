from impl import kfilter
import numpy as np
import matplotlib.pyplot as plt

time = 0
accel = -5
position = 0
velocity = 0
step = 0.00416666666
time_values = []
altitude_values = []
kalman_altitude_values = []
velocity_values = []
kalman_velocity_values = []
accel_values = []
kalman_accel_values = []
for i in range(160):
    for i2 in range(12):

        position += (velocity * step) + (0.5*accel*(step**2))
        velocity += (accel * step)

        time += step

        kfilter.predict()
        if i2 == 0:
            kfilter.update(np.array([np.random.normal(position, .5), np.random.normal(accel, 0.2)]))
        else:
            kfilter.update2(np.array([np.random.normal(accel, 0.2)]))

        time_values.append(time)
        altitude_values.append(position)
        kalman_altitude_values.append(kfilter.x[0]-position)
        velocity_values.append(velocity)
        kalman_velocity_values.append(kfilter.x[1]-velocity)
        accel_values.append(accel)
        kalman_accel_values.append(kfilter.x[2])


plt.subplot(3, 1, 1)
plt.plot(time_values, kalman_altitude_values)
plt.ylabel('Altitude (m)')
plt.xlabel('Time (s)')

plt.subplot(3, 1, 2)
plt.plot(time_values, kalman_velocity_values)
plt.ylabel('Velocity (m/s)')
plt.xlabel('Time (s)')

plt.subplot(3, 1, 3)
plt.plot(time_values, accel_values)
plt.plot(time_values, kalman_accel_values)
plt.ylabel('Acceleration (m/s^2)')
plt.xlabel('Time (s)')

plt.show()
