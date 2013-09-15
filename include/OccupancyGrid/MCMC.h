#include "OccupancyGrid.h"
OccupancyGrid::Marginals runSlowMetropolis(const OccupancyGrid &occupancyGrid, size_t iterations, double max_clock);
OccupancyGrid::Marginals runDDMCMC(const OccupancyGrid &occupancyGrid, size_t iterations, double max_clock);
