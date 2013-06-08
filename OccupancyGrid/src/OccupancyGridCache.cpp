#include "../include/OccupancyGridCache.h"
#include "../include/visualiser.h"

#undef DEBUG

using namespace gtsam;
using namespace std;

/******************************************************************************/
double OccupancyGridCache::operator()(
    const LaserFactor::Occupancy &occupancy
    ) const 
{
  if (lastOccupancy_.empty()) {
    lastEnergy_.resize(factors_.size());
    for (Index i = 0; i < factors_.size(); i++)
      lastEnergy_[i] = laserFactorValue(i, occupancy);

  } else {

    // We already have computed all the factors.
    // Only recompute the factors that have their cells changed.
    // loop over all the cells
    std::vector<bool> recompute_factors(factors_.size(), false);
    size_t factors_to_recompute = 0;
    for (Index i = 0; i < cell2factors_.size(); i++) {
      // check if the celll has changed
      if (lastOccupancy_.at(i) != occupancy.at(i)) {
        // get the related factors that need to be recomputed
        std::vector<gtsam::Index> cellfactors = cell2factors_[i];
#ifdef DEBUG
        std::cout << "Occupancy at " << i << " changed" << endl;
        std::cout << "Cell at " << i << " has " << cellfactors.size() << " factors" << endl;
#endif // DEBUG
        for (Index j = 0; j < cellfactors.size(); j++) {
          gtsam::Index factoridx = cellfactors[j];
          recompute_factors[factoridx] = true;
          factors_to_recompute ++;
        }
      }
    }
#ifdef DEBUG
    std::cout << "Recomputing " << factors_to_recompute << " factors" << endl;
#endif // DEBUG

    // Recompute the factors that need to be recomputed
    
    for (Index i = 0; i < factors_.size(); i++) {
      if (recompute_factors[i]) {
        lastEnergy_[i] = laserFactorValue(i, occupancy);
#ifdef DEBUG
      } else {
        // Energy should remain same for all other factors.
        if ( lastEnergy_[i] != laserFactorValue(i, occupancy) ) {
          cout << "Energy changed for factor:" << i << endl;
          boost::shared_ptr<LaserFactor> factor =
            boost::dynamic_pointer_cast<LaserFactor, DiscreteFactor>(
                factors_[i]);
          vector<Index> fcells = factor->cells_;
          for (Index j = 0; j < fcells.size(); j ++ ) {
            cout << "Checking occupancy at " << fcells[j] << endl;
            if (occupancy.at(fcells[j]) != lastOccupancy_.at(fcells[j])) {
              cout << "Occupancy at cell " << fcells[j] << " has changed at " << j << "/" << fcells.size() << endl;
              assert(false);
            }
          }
          cout << "Occupancies are all the same" << endl;
          assert(false);
        }
#endif // DEBUG
      }
    }
  }

  // sum up the energies and return
  double sum = 0;
  for (int i = 0; i < lastEnergy_.size(); i++) {
    sum += lastEnergy_[i];
  }

  lastOccupancy_ = occupancy; // creates a copy of the occupancy 
  return sum;
}

/******************************************************************************/
void OccupancyGridCache::addLaser(const gtsam::Pose2 &pose, double range) {
  gtsam::Index factoridx;
  vector<Index> fcells; // list of keys of cells hit by the laser
  OccupancyGrid::addLaserReturn(pose, range, factoridx, fcells);
#ifdef DEBUG_INPUT
  cout << "Laser " << factoridx << " passed through " << fcells.size() << " cells" << endl;
  global_vis_.highlightCells(fcells);
  global_vis_.enable_show();
  global_vis_.show(1);
#endif
  for (int i = 0; i < fcells.size(); i++) {
    gtsam::Index cellidx = fcells[i];
    cell2factors_[cellidx].push_back(factoridx);
  }
}
