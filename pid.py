from matplotlib import pyplot as plt


# TODO: Better documentation for this class
class PIDController:
    times = []
    errors = []

    def __init__(self, target, kp, ki, kd):
        self._target = target
        self._error = lambda x: x - target
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def update(self, time, control_input, process_state):
        self.times.append(time)
        error = self._error(process_state)
        self.errors.append(error)

        if (len(self.times) > 1):
            return control_input + self._p(error) + self._i(error) + self._d(error)
        else:
            return control_input

    def _p(self, error):
        return self.kp * error

    def _i(self, error):
        return self.ki * sum(self.errors)

    def _d(self, error):
        # rate of change between the past two errors
        derivative = ((self.errors[-1]-self.errors[-2]) / (self.times[-1]-self.times[-2]))
        return self.kd * derivative

    def figure(self):
        plt.figure('PID Controller')
        plt.plot(self.times, self.errors)
        plt.ylabel('Error (m)')
        plt.xlabel('Time (s)')
        #  Do not show figures
