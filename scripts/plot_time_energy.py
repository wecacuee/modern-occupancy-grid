#!/usr/bin/python
from script_utils import load_time_energy
import matplotlib.pyplot as plt
import numpy as np
import sys
import imp

from matplotlib import rc
rc('font',**{'family':'serif','serif':['Times']})
rc('text', usetex=True) 

def _plot_time_energy_two_assumption(times, energy):
    times = np.hstack((times, [200]))
    energy = np.hstack((energy, energy[-1]))
    return plt.plot(times, energy)

if __name__ == '__main__':
    conf = sys.argv[1]
    plt.figure(figsize=(1.618*4, 4))

    plot_obj = []
    legend_names = []
    config = imp.load_source('config', conf)
    for exe in config.executables():
        times, energy = load_time_energy( open( exe.plotdatafname()))
        if exe.exe() == "bin/TwoAssumptionAlgorithm":
            p, = _plot_time_energy_two_assumption(times, energy)
        else:
            p, = plt.plot(times, energy)
        plot_obj.append(p)
        legend_names.append(exe.legend())
    plt.legend(plot_obj, legend_names)
    ax = plt.gca()
    ax.set_ylim([2.78e8, 2.90e8])
    ax.set_xlabel("Time (clock seconds)")
    ax.set_ylabel("Total energy of the graph")
    #plt.savefig('doc/figures/relativeconvergence.pdf')
    plt.show()
