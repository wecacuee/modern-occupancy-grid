#ifndef __OCCUPANCY_GRID__
#define __OCCUPANCY_GRID__

/**
 *      @file OccupancyGrid.h
 *      @date May 23, 2012
 *      @author Brian Peasley
 *      @author Frank Dellaert
 */


#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/discrete/DiscreteSequentialSolver.h>

#include <CppUnitLite/TestHarness.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include "/home/brian/Libraries/OccupancyGrid/OccupancyGrid/include/LaserFactor.h"

#include <vector>
#include <stdlib.h>
#include <math.h>

using namespace std;
using namespace gtsam;



/**
 * OccupancyGrid Class
 * An occupancy grid is just a factor graph.
 * Every cell in the occupancy grid is a variable in the factor graph.
 * Measurements will create factors, as well as the prior.
 */
class OccupancyGrid : public DiscreteFactorGraph {
private:
	size_t			width_;		//number of cells wide the grid is
	size_t			height_;	//number of cells tall the grid is
	double			res_;		//the resolution at which the grid is created

	vector<Index> 	cells_;			//list of keys of all cells in the grid
	vector<Index> 	laser_indices_; //indices of the laser factor in factors_
	vector<Index>	heat_map_;		//heat map of the occupancy grid

	vector<Pose2>	pose_;		//list of poses of the added lasers
	vector<double>	range_;		//list of ranges of the added lasers

public:

	size_t width() const {
		return width_;
	}
	size_t height() const {
		return height_;
	}

	typedef Values Occupancy;
	typedef std::vector<Index> HeatMap;
	typedef std::vector<double> Marginals;

	///constructor
	///Creates a 2d grid of cells with the origin in the center of the grid
	OccupancyGrid(double width, double height, double resolution);

	/// Returns an empty occupancy grid of size width_ x height_
	Occupancy emptyOccupancy() const;

	/// Returns a heat map
	HeatMap heatMap() const { return heat_map_; }

	///add a prior
	void addPrior(Index cell, double prior);

	///add a laser measurement
	void addLaser(const Pose2 &pose, double range);

	/// returns the number of cells in the grid
	inline size_t cellCount() const {
		return width_*height_;
	}

	/// returns the key of the cell in which point (x,y) lies.
	Index keyLookup(double x, double y) const;

	/**
	 * @brief Computes the value of a laser factor
	 * @param index defines which laser is to be used
	 * @param occupancy defines the grid which the laser will be evaulated with
	 * @ret a double value that is the value of the specified laser factor for the grid
	 */
	double laserFactorValue(Index index, const Occupancy &occupancy) const{
		return (*factors_[ laser_indices_[index] ])(occupancy);
	}

	/// returns the sum of the laser factors for the current state of the grid
	double operator()(const Occupancy &occupancy) const;

	/// run a metropolis sampler
	Marginals runMetropolis(size_t iterations);

	/// save lasers to file
	void saveLaser(const char *fname) const;

	/// save heat map as a pgm file
	void saveHeatMap(const char *fname) const;

};



#endif
