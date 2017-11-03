from matplotlib import pyplot as plt

class PIDTargetController:
    time_vals = []
    est_trg = []
    last_err = 0

    def __init__(self, target, k, tau_i): # tau_i (sec/repeat)
        self._trg = target
        self._err = lambda x: x - target
        self.k = k
        self.tau_i = tau_i

    def tck(self, time, est_trg):
        self.time_vals.append(time)
        self.est_trg.append(est_trg)
        err = self._err(est_trg)
        return self._p(err) + self._id(err)

    def _p(self, err):
        return self.k * err

    def _id(self, err):
        try:
            ki = self.k / self.tau_i
            id = ki * (err - self.last_err)
            self.last_err = err
            return id
        except ZeroDivisionError:
            pass # Don't do anything if tau_i is 0

    def figure(self):
        plt.figure('PID Controller')
        plt.plot(self.time_vals, [self._trg]*len(self.time_vals), label='Target')
        plt.plot(self.time_vals, self.est_trg, label='Estimation')
        plt.ylabel('Target (m)')
        plt.xlabel('Time (s)')
        plt.legend()
        #  Do not show figures
