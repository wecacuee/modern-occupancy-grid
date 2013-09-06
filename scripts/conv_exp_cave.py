import convergence_experiment
MAP_SIZE = 18 # in m
NCELLS = 100
CELLSIZE = MAP_SIZE / NCELLS

def executables():
    return convergence_experiment.executables(
        MAP_SIZE, MAP_SIZE, "Data/cave_player/")

