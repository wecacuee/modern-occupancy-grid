import os
import numpy as np

def save_time_energy(fd, times, energy):
    np.save(fd, times)
    np.save(fd, energy)

def load_time_energy(fd):
    times = np.load(fd)
    energy = np.load(fd)
    return times, energy

class ExecutableConfig(object):
    def __init__(self, exe, conf):
        self._exe = exe
        self._conf = conf

    def exe(self):
        return self._exe

    def legend(self):
        return self._conf['legend']

    def key(self):
        return os.path.basename(self._exe)

    def plotdatafname(self):
        return "Data/%s-plot-time-energy.npy" % self.key()

def key(executable):
    return executable.key()

def plotdatafname(executable):
    return executable.plotdatafname()

