#pragma once
#include "OccupancyGrid/OccupancyGrid.h"
#include <boost/program_options.hpp>
OccupancyGrid loadOccupancyGrid(boost::program_options::variables_map vm);
