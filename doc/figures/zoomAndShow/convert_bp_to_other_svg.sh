#!/bin/bash
cat zoomAndShowBP.svg | sed -e 's/run_belief_propagation/TwoAssumptionAlgo/g' > zoomAndShowTwoAssumptionAlg.svg
cat zoomAndShowBP.svg | sed -e 's/run_belief_propagation/SICKSlowMetropolis/g' > zoomAndShowMetropolis.svg
cat zoomAndShowBP.svg | sed -e 's/run_belief_propagation/SICKDDMCMC/g' > zoomAndShowSICKDDMCMC.svg
cat zoomAndShowBP.svg | sed -e 's/run_belief_propagation/dualdecomposition/g' > zoomAndShowDD.svg
