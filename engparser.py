from const import THRUST_SCALE

class NARThrustParser:
    def __init__(self, eng_file, thrust_scl=THRUST_SCALE):
        self.thrust_scl = thrust_scl
        self._tdata = [tuple(float(x) for x in line.split(' ')) \
                        for line in eng_file.readlines() if line[0] != ';']

    @property
    def _data_tuple(self):
        return self._tdata

    @property
    def time(self):
        return tuple(x[0] for x in self._data_tuple)

    @property
    def thrust(self):
        return tuple(x[1]*self.thrust_scl for x in self._data_tuple)
