from __future__ import division
from script_utils import ExecutableConfig

def exec_config():
    args = [
        ("bin/run_belief_propagation",
         {"legend" : "Belief Propagation"}),
        ("bin/SICK_DDMCMC", 
         {"legend" : "Metropolis Hastings with heatmap"}),
        ("bin/SICKSlowMetropolis",
         {"legend" : "Metropolis Hastings"}),
        ("bin/dualdecomposition",
         {"legend" : "Dual Decomposition"}),
        ("bin/TwoAssumptionAlgorithm",
         {"legend" : "Two Assumption Algorithm"})
    ] 
    return args
def executables(dir_):
    args = exec_config()
    for k, conf in args:
        conf['dir'] = dir_
        conf['args'] = [.18, "--dir=%s" % dir_]
    return [ExecutableConfig(*args) for args in args]

