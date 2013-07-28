/**
 *      @file OccupancyGridCache.h
 *      @date June 6, 2013
 *      @author Vikas Dhiman
 */
#pragma once
#include "OccupancyGrid.h"

/**
 * We do not need to recompute all laser factors when we flip only one or few
 * of the cells. This cache class provide that behaviour
 *
 */
class OccupancyGridCache : public OccupancyGrid {
public:
  typedef boost::shared_ptr< LaserFactor > LaserFactorPtr;

  /** 
   * @brief: Initialize the cache
   * @param og: The occupancy grid to be wrapped
   * @param cell_count: The cell count to initialize the cache.
   */
  OccupancyGridCache(double width, double height, double resolution):
    OccupancyGrid(width, height, resolution), cell2factors_(cellCount()) {}

  /**
   * @brief: Add laser to the cache
   * @param factor : LaserFactor to be added to cache.
   */
  virtual void addLaser(const gtsam::Pose2 &pose, double range);

  /**
   * @brief: Use caching to compute energy of the factor graph efficiently
   * 
   * For the first time all the energy for all laser factors are computed but
   * in the next iteration only those factors are computed whose value has
   * changed from the last occupancy.
   *
   * @param occupancy: If a cells is occupied or free.
   */
  virtual double operator()(const LaserFactor::Occupancy &occupancy) const;

private:
  /// the factors that are influenced by a cell
  // a vector of factor indices for each cell in the grid
  std::vector<std::vector< gtsam::Index > > cell2factors_; 

  /// Occupancy provided last time for computing energy
  mutable LaserFactor::Occupancy lastOccupancy_;               

  /// Energy contributed by each laser factor
  mutable std::vector<gtsam::Index> lastEnergy_;
};
