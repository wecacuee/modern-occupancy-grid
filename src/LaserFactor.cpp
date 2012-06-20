/**
 *      @file LaserFactor.cpp
 *      @date May 23, 2012
 *      @author Brian Peasley
 *      @author Frank Dellaert
 */

#include "../include/LaserFactor.h"

#include <vector>
#include <stdlib.h>
#include <math.h>

/**
 * Find value for given assignment of values to variables
 * return 1000 if any of the non-last cell is occupied and 1 otherwise
 * Values contains all occupancy values (0 or 1)
 */
double LaserFactor::operator()(const Values &vals) const{

	// loops through all but the last cell and checks that they are all 0.  Otherwise return 1000.
	for(Index i = 0; i < cells_.size() - 1; i++){
		if(vals.at(cells_[i]) == 1)
			return 1000;
	}

	// check if the last cell hit by the laser is 1.  return 900 otherwise.
	if(vals.at(cells_[cells_.size() - 1]) == 0)
		return 900;

	return 1;


}
