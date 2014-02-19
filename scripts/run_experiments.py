#!/usr/bin/python
from __future__ import division
import subprocess
from multiprocessing import Pool
import imp
import sys

def run(executable):
    cmd = [str(i) for i in ["time", "-p"] + executable.cmd()]
    print(" ".join(cmd))
    return subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE)

def run_plot(executable):
    pipe = run(executable)
    executable.process(pipe.stdout)

def main(conf, toexec=None):
    pool = Pool(processes=(5 if (toexec is None) else 1))
    config = imp.load_source('config', conf)
    for exe in config.executables():
        if (toexec is None) or (toexec in exe.exe()):
            pool.apply_async(run_plot, args=(exe,))
    pool.close()
    pool.join()

if __name__ == '__main__':
    if len(sys.argv) > 2:
        toexec = sys.argv[2]
    else:
        toexec = None
    main(sys.argv[1], toexec)
