import os
import numpy as np

def save_time_energy(fd, times, energy):
    np.save(fd, times)
    np.save(fd, energy)

def load_time_energy(fd):
    times = np.load(fd)
    energy = np.load(fd)
    return times, energy

def grep_energy_lines(file):
    return (line for line in file
            if "<Energy>" in line)

def parse_time_energy(lines):
    """
    >>> parse_time_energy(["<Energy>\t10.2\t1e8", "<Energy>\t12.3\t2e7"])
    (array([10.2, 12.3]), array([1e8, 2e7]))
    """
    times = []
    energy = []
    for li in lines:
        _, t, e = li.strip().split()
        times.append(float(t))
        energy.append(float(e))
    return np.array(times), np.array(energy)

class TimeEnergyProcessor(object):
    def process(self, executable, stdout):
        times, energy = parse_time_energy(grep_energy_lines(stdout))
        save_time_energy(open(executable.plotdatafname(), 'w'), times, energy)

class ExecutableConfig(object):
    def __init__(self, exe, conf, processor=TimeEnergyProcessor()):
        self._exe = exe
        self._conf = conf
        self._processor = processor

    def exe(self):
        return self._exe

    def legend(self):
        return self._conf['legend']

    def key(self):
        return os.path.basename(self._exe)

    def plotdatafname(self):
        return "%s/%s-plot-time-energy.npy" % (self._conf['dir'], self.key())

    def cmd(self):
        return [self._exe] + self._conf["args"]

    def process(self, stdout):
        self._processor.process(self, stdout)

def key(executable):
    return executable.key()

def plotdatafname(executable):
    return executable.plotdatafname()

