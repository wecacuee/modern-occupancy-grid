#!/usr/bin/python
from __future__ import division
import subprocess
import numpy as np
from script_utils import (save_time_energy, plotdatafname)
from convergence_experiment import executables
from multiprocessing import Pool

MAP_SIZE = 18 # in m

def run(executable, resolution):
    cmd = [str(i) 
         for i in ["time", "-p", executable, MAP_SIZE, MAP_SIZE, MAP_SIZE / resolution]]
    print(" ".join(cmd))
    return subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE)

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


def run_plot(executable, resolution):
    pipe = run(executable.exe(), resolution)
    times, energy = parse_time_energy(grep_energy_lines(pipe.stdout))
    save_time_energy(open(plotdatafname(executable), 'w'), times, energy)

def main(toexec=None):
    resolution = 100
    pool = Pool(processes=(5 if (toexec is None) else 1))
    for exe in executables():
        if (toexec is None) or (toexec in exe.exe()):
            pool.apply_async(run_plot, args=(exe, resolution))
    pool.close()
    pool.join()

if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        toexec = sys.argv[1]
    else:
        toexec = None
    main(toexec)
