from __future__ import division
import convergence_experiment
from script_utils import ExecutableConfig

def executables():
    args = convergence_experiment.exec_config()
    for k, conf in args:
        dir_ = "Data/albertb.sm/"
        conf['dir'] = dir_
        conf['args'] = [.05, "--width=30", "--height=30",
                        "--dir=%s" % dir_]
    return [ExecutableConfig(*args) for args in args]

