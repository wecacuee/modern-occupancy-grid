/**
 *      @file LaserFactor.cpp
 *      @date May 23, 2012
 *      @author Brian Peasley
 *      @author Frank Dellaert
 */

#include "OccupancyGrid/LaserFactor.h"
#include "OccupancyGrid/visualiser.h"

#include <vector>
#include <stdlib.h>
#include <math.h>

#include <boost/bind.hpp>
#include <boost/bind/placeholders.hpp>

using namespace std;
using namespace gtsam;

#define LASER_FACTOR_DEBUG 0

double LaserFactor::operator()(const Values &vals) const {
#if LASER_FACTOR_DEBUG
  global_vis_.highlightCells(cells_);
  global_vis_.show();
#endif
  if (is_same_assignment(
        boost::bind(&LaserFactor::w0_assignment, this, _1), vals))
    return w0_;
  else if (
      reflectance_ && // w1 assignment is not valid for reflectance case
      is_same_assignment(
        boost::bind(&LaserFactor::w1_assignment, this, _1), vals))
    return w1_;
  else
    return w_otherwise_;
}
