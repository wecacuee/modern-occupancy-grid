#include "OccupancyGrid/TwoAssumptionAlgorithm.h"

#include "OccupancyGrid/OccupancyGrid.h"
#include "OccupancyGrid/LaserFactor.h"

#include <vector>


using namespace gtsam;
using namespace occgrid;

typedef boost::shared_ptr< LaserFactor > LaserFactorPtr;

void two_assumption_algorithm(const OccupancyGrid& occupancyGrid,
    LaserFactor::Occupancy& best_assign,
    std::vector<double>& energy) {
  double LOG_ODDS_OCCUPIED(1.3863),
         LOG_ODDS_FREE(-1.3863);

  for (gtsam::Index f = occupancyGrid.cellCount(); f < occupancyGrid.factorCount() + occupancyGrid.cellCount(); ++f) {
    LaserFactorPtr lf = occupancyGrid.factorFromNodeId(f);
    BOOST_FOREACH(gtsam::Index x, lf->cells_)
      energy[x] += LOG_ODDS_FREE;
    size_t last_cell_idx = lf->cells_[lf->cells_.size() - 1];
    energy[last_cell_idx] -= LOG_ODDS_FREE;
    energy[last_cell_idx] += LOG_ODDS_OCCUPIED;
  }

  for (gtsam::Index x = 0; x < occupancyGrid.cellCount(); ++x) {
    best_assign[x] = (energy[x] < 0) ? 0 : 1;
  }
}
