from matplotlib import pyplot as plt

class PIDController:
    """PID Controller.
    """
    times = []
    errors = []

    def __init__(self, target, proportional_gain, integral_gain, derivative_gain):
        self._target = target
        self._error = lambda x: x - target
        self.prop_gain = proportional_gain
        self.int_gain = integral_gain
        self.deriv_gain = derivative_gain

    def update(self, time, process_state):
        """Calculates PID value for given reference feedback.

        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d \frac{de}{dt}
        """
        self.times.append(time)
        self.errors.append(self._error(process_state))

        if len(self.times) > 1:
            return self._p() + self._i() + self._d()
        return self._p() + self._i()

    def _p(self):
        """Calculates proportional value.

        .. math:
            u(t) = K_p e(t)
        """
        return self.prop_gain * self.errors[-1]

    def _i(self):
        """Calculates integral value.

        .. math:
            u(t) = K_i \sum e(t)
        """
        return self.int_gain * sum(self.errors)

    def _d(self):
        """Calculates derivative value.

        .. math:
            u(t) = K_d \frac{de}{dt}
        """
        # Rate of change between the past two errors
        derivative = ((self.errors[-1]-self.errors[-2]) / (self.times[-1]-self.times[-2]))
        return self.deriv_gain * derivative

    def figure(self, title=None):
        """Plot the residuals of the process value and the target.
        Invoke `pyplot.show()` for the figure to be shown.

        Math:
            err = estimation - target
        """
        plt.figure(title or 'PID Controller Error')
        plt.plot(self.times, self.errors)
        plt.ylabel('Error (m)')
        plt.xlabel('Time (s)')
        #  Do not show figures
