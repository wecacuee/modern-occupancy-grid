/**
 *      @file OccupancyGrid.h
 *      @date May 23, 2012
 *      @author Brian Peasley
 *      @author Frank Dellaert
 */

#pragma once

#include "LaserFactor.h"

#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/discrete/DiscreteSequentialSolver.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/shared_ptr.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>

#include <vector>
#include <cstdlib>
#include <cmath>
#include <cassert>

#define DEBUG 1

/**
 * OccupancyGrid Class
 * An occupancy grid is just a factor graph.
 * Every cell in the occupancy grid is a variable in the factor graph.
 * Measurements will create factors, as well as the prior.
 */
class OccupancyGrid : public gtsam::DiscreteFactorGraph {
  friend class LaserFactor;
  typedef boost::shared_ptr< LaserFactor > LaserFactorPtr;
  typedef boost::shared_ptr< gtsam::DiscreteFactor > DiscreteFactorPtr;
  typedef LaserFactor::Occupancy Occupancy;
  typedef LaserFactor::Occupancy::key_type IndexType;
  typedef LaserFactor::Occupancy::mapped_type ValueType;

private:

  size_t			width_;  ///< number of cells wide the grid is
	size_t			height_; ///< number of cells tall the grid is
	double			res_;    ///< the resolution at which the grid is created

	std::vector<gtsam::Index>	heat_map_;      ///< heat map of the occupancy grid
 
	std::vector<gtsam::Pose2>	pose_; ///< list of poses of the added lasers
	std::vector<double>	range_;      ///< list of ranges of the added lasers

  /// the factors that are influenced by a cell
  // a vector of factor indices for each cell in the grid
  std::vector<std::vector<gtsam::Index> > cell2factors_; 

  /// Ray trace and return the cells traversed and key of last cell
  void rayTrace(const gtsam::Pose2 &pose, const double range,
    std::vector<gtsam::Index>& cells,
    gtsam::Index& key);

  /// add laser and return the index of new factor and the cells traversed
  LaserFactorPtr addLaserReturn(const gtsam::Pose2 &pose, double range,
      gtsam::Index& factor_index,
      std::vector<gtsam::Index>& cells);


  /// Generate a node id for this factor
  gtsam::Index addNode(LaserFactorPtr f) {
    factors_.push_back(f);
    gtsam::Index factor_idx = factors_.size() - 1;
    gtsam::Index node_idx = factor_idx + cellCount();
    return node_idx;
  }

public:

	size_t width() const {
		return width_;
	}
	size_t height() const {
		return height_;
	}

	typedef std::vector<gtsam::Index> HeatMap;
	typedef std::vector<double> Marginals;

	/**
	 * Constructor
	 * Creates a 2d grid of cells with the origin in the center of the grid
	 */
	OccupancyGrid(double width, double height, double resolution);

	/// Returns an empty occupancy grid of size width_ x height_
	LaserFactor::Occupancy emptyOccupancy() const;

	/// Returns a heat map
	HeatMap heatMap() const { return heat_map_; }

	/// Add a prior
	void addPrior(gtsam::Index cell, double prior);

	/// Add a laser measurement
	virtual void addLaser(const gtsam::Pose2 &pose, double range);

	/// Returns the number of cells in the grid
	inline size_t cellCount() const {
		return width_*height_;
	}

	/// Returns the key of the cell in which point (x,y) lies.
	gtsam::Index keyLookup(double x, double y) const;

	/**
	 * @brief Computes the value of a laser factor
	 * @param index defines which laser is to be used
	 * @param occupancy defines the grid which the laser will be evaulated with
	 * @ret a double value that is the value of the specified laser factor for the grid
	 */
	double laserFactorValue(const gtsam::Index node_idx,
      const LaserFactor::Occupancy &occupancy) const 
  {
		return (*factors_[node_idx - cellCount()])(occupancy);
	}

	/// returns the sum of the laser factors for the current state of the grid
	virtual double operator()(const LaserFactor::Occupancy &occupancy) const;

  /** 
   * Returns the change in Energy value occupancy[cellidx] is replaced by
   * newValue.
   *
   * occupancy remains unchanged, when the function returns. However it is
   * changed intermediately, so make sure you add locking while making the
   * algorithm multi-threaded.
   */
  double computeDelta(LaserFactor::Occupancy &occupancy, const gtsam::Index &cellidx, size_t newValue) const;

  inline bool is_factor(gtsam::Index node_idx) const {
    return (node_idx < cellCount()) ? false : true;
  }

  inline LaserFactorPtr factorFromNodeId(gtsam::Index node_idx) const {
    return boost::dynamic_pointer_cast<LaserFactor>(factors_[node_idx - cellCount()]);
  }

  /// Returns the neighbors of a cell in a factor graph
  inline const std::vector< gtsam::Index >& getNeighbors(gtsam::Index node_idx) const {
    return (is_factor(node_idx)) ? 
      factorFromNodeId(node_idx)->cells_ : cell2factors_[node_idx];
  }

  /// Returns the number of factors
  inline size_t factorCount() const {
    return factors_.size();
  }


	/// run a metropolis sampler
	Marginals runMetropolis(size_t iterations);

	/// save lasers to file
	void saveLaser(const char *fname) const;

	/// save heat map as a pgm file
	void saveHeatMap(const char *fname) const;
};

