from __future__ import division
import convergence_experiment
from script_utils import ExecutableConfig

def executables():
    args = convergence_experiment.exec_config()
    for k, conf in args:
        conf['args'] = [.05, "--width=25", "--height=25",
                        "--dir=Data/albert.sw/"]
    return [ExecutableConfig(*args) for args in args]

