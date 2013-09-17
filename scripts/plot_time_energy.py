#!/usr/bin/python
from script_utils import load_time_energy
import matplotlib.pyplot as plt
import numpy as np
import sys
import imp

from matplotlib import rc
rc('font',**{'family':'serif','serif':['Times']})
rc('text', usetex=True) 

def _plot_time_energy_two_assumption(exe, times, energy):
    # times = np.hstack((times, [200]))
    # energy = np.hstack((energy, energy[-1]))
    return plt.plot(times, energy, '-o', label=exe.legend())

if __name__ == '__main__':
    conf = sys.argv[1]
    fig = plt.figure(figsize=(1.618*4, 4))
    fig.subplots_adjust(bottom=0.16)

    config = imp.load_source('config', conf)
    for exe in config.executables():
        times, energy = load_time_energy( open( exe.plotdatafname()))
        if "albertb" in exe.dir():
            before20 = (times < 20000)
            times = times[before20]
            energy = energy[before20]
        if exe.exe() == "bin/TwoAssumptionAlgorithm":
            p, = _plot_time_energy_two_assumption(exe, times, energy)
        else:
            p, = plt.plot(times, energy, label=exe.legend())
    ax = plt.gca()
    if "cave" in exe.dir() or "step" in conf:
        leg = ax.legend(loc='best', fancybox=True)
        leg.get_frame().set_alpha(0.5)
    if "step" in conf:
        ax.set_ylim([2.78e8, 2.90e8])
    ax.set_xlabel("Time (clock seconds)")
    ax.set_ylabel("Total energy of the graph")
    plt.savefig(exe.plotfigurefname())
    plt.show()
