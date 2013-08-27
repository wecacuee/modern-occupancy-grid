from script_utils import ExecutableConfig

def executables():
    return [ExecutableConfig(*args) for args in
            [
            ("bin/run_belief_propagation", {"legend" : "Belief Propagation"}),
            ("bin/SICK_DDMCMC", {"legend" :
                                 "Metropolis Hastings with heatmap"}),
            ("bin/SICKSlowMetropolis", {"legend" :
                                 "Metropolis Hastings"}),
            ("bin/dualdecomposition", {"legend" : "Dual Decomposition"}),
            ("bin/TwoAssumptionAlgorithm", {"legend" : "Two Assumption Algorithm"})
            ] ]

