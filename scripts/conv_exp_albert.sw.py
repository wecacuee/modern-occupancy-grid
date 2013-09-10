from __future__ import division
import convergence_experiment
from script_utils import ExecutableConfig

def executables():
    args = convergence_experiment.exec_config()
    for k, conf in args:
        conf['args'] = [.05, "--width=30", "--height=30",
                        "--dir=Data/albertb.sm/"]
    return [ExecutableConfig(*args) for args in args]

