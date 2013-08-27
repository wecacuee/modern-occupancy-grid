#include "OccupancyGrid/occgrid.h"
template <>
const uint8_t OccupancyGrid2D<double, int>::OCCUPIED = 0;

template <>
const uint8_t OccupancyGrid2D<double, int>::FREE = 255;
