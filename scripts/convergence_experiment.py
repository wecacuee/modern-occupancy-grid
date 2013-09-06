from __future__ import division
from script_utils import ExecutableConfig

MAP_SIZE = 18 # in m
NCELLS = 100
CELLSIZE = MAP_SIZE / NCELLS

def executables():
    return [ExecutableConfig(*args) for args in
            [
            ("bin/run_belief_propagation",
             {"legend" : "Belief Propagation",
              "args" : [MAP_SIZE, MAP_SIZE, CELLSIZE] }),
            ("bin/SICK_DDMCMC", 
             {"legend" : "Metropolis Hastings with heatmap",
              "args" : [MAP_SIZE, MAP_SIZE, CELLSIZE] }),
            ("bin/SICKSlowMetropolis",
             {"legend" : "Metropolis Hastings",
              "args" : [MAP_SIZE, MAP_SIZE, CELLSIZE] }),
            ("bin/dualdecomposition",
             {"legend" : "Dual Decomposition",
              "args" : [MAP_SIZE, MAP_SIZE, CELLSIZE] }),
            ("bin/TwoAssumptionAlgorithm",
             {"legend" : "Two Assumption Algorithm",
              "args" : [MAP_SIZE, MAP_SIZE, CELLSIZE] }),
            ] ]

