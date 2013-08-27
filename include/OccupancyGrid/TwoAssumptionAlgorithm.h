#pragma once
#include "OccupancyGrid/OccupancyGrid.h"
#include "OccupancyGrid/LaserFactor.h"
#include "gtsam/discrete/Assignment.h"
#include "gtsam/base/types.h"
void two_assumption_algorithm(const OccupancyGrid& occupancyGrid,
    LaserFactor::Occupancy& best_assign,
    std::vector<double>& energy);
