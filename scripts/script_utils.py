import os
import numpy as np

def key(executable):
    return os.path.basename(executable)

def plotdatafname(executable):
    return "Data/%s-plot-time-energy.npy" % key(executable)

def save_time_energy(fd, times, energy):
    np.save(fd, times)
    np.save(fd, energy)

def load_time_energy(fd):
    times = np.load(fd)
    energy = np.load(fd)
    return times, energy

def executables_and_config():
    return [
        ("bin/run_belief_propagation", {"name" : "Belief Propagation"}),
        ("bin/SICK_DDMCMC", {"name" : "Metropolis hastings with heatmap" }),
        ("bin/SICKSlowMetropolis", {"name" : "Metropolis hastings"}),
        ("bin/dualdecomposition", {"name" : "Dual Decomposition"}),
        ("bin/TwoAssumptionAlgorithm", {"name": "Two assumption"})
    ]

