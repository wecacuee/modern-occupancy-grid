from __future__ import division
from script_utils import ExecutableConfig

def executables(width, height, dir):
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
    for k, conf in args:
        conf['args'] = [width, height, .18, "--dir=%s" % dir]
    return [ExecutableConfig(*args) for args in args]

