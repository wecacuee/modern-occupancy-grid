from __future__ import division
import script_utils

MAP_SIZE = 18 # in m
NCELLS = 100
CELLSIZE = MAP_SIZE / NCELLS
DUALDECOMPOSITION = "bin/dualdecomposition"

class ExecutableConfig(script_utils.ExecutableConfig):
    def plotdatafname(self):
        return "Data/%s-plot-time-energy-step=%d.npy" % (
            self.key(), self._conf["stepsize"])

    def plotfigurefname(self):
        return "Data/plot-time-energy-step.pdf"

def executables():
    return [ExecutableConfig( 
            DUALDECOMPOSITION,
             {"legend" : "Step size=%d" % step,
              "stepsize" : step,
              "dir" : "Data/",
              "args" : [MAP_SIZE, MAP_SIZE, CELLSIZE, "--step=%d" % step] })
            for step in [20, 50, 100, 200, 500] ]
