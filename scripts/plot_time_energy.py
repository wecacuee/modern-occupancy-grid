#!/usr/bin/python
from script_utils import (load_time_energy, plotdatafname,
                          executables_and_config)
import matplotlib.pyplot as plt
import numpy as np

from matplotlib import rc
rc('font',**{'family':'serif','serif':['Times']})
rc('text', usetex=True) 

def _plot_time_energy_two_assumption(times, energy):
    times = np.hstack((times, [200]))
    energy = np.hstack((energy, energy[-1]))
    return plt.plot(times, energy)

if __name__ == '__main__':
    plt.figure(figsize=(1.618*5, 5))

    plot_obj = []
    legend_names = []
    for exe, conf in executables_and_config():
        times, energy = load_time_energy( open( plotdatafname(exe)))
        if exe == "bin/TwoAssumptionAlgorithm":
            p, = _plot_time_energy_two_assumption(times, energy)
        else:
            p, = plt.plot(times, energy)
        plot_obj.append(p)
        legend_names.append(conf['name'])
    plt.legend(plot_obj, legend_names)
    ax = plt.gca()
    ax.set_xlabel("Time (clock seconds)")
    ax.set_ylabel("Total energy of the graph")
    #plt.savefig('doc/figures/relativeconvergence.pdf')
    plt.show()
