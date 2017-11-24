import numpy as np


class RocketEngine:
    """Models rocket engine using an NAR thrust curve.
    """
    def __init__(self, eng_file, thrust_scale):
        self.thrust_scale = thrust_scale
        self._thrust_data = [tuple(float(x) for x in line.split(' ')) \
                             for line in eng_file.readlines() if line[0] not in (';', ' ')]

    @property
    def _data_tuple(self):
        return self._thrust_data

    @property
    def raw_times(self):
        return tuple(x[0] for x in self._data_tuple)

    @property
    def raw_thrusts(self):
        return tuple(x[1]*self.thrust_scale for x in self._data_tuple)

    # Get engine thrust at a specified time
    def thrust(self, time):
        # Raw thrust values plus interpolation
        return np.interp(time, self.raw_times, self.raw_thrusts, right=0)
