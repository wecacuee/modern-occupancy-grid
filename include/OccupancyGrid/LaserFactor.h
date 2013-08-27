/**
 *      @file LaserFactor.h
 *      @author Brian Peasley
 */

#pragma once

#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/geometry/Pose2.h>

#include <CppUnitLite/TestHarness.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>

#include <boost/iterator/filter_iterator.hpp>
#include <boost/utility/result_of.hpp>
#include <boost/foreach.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/typeof/typeof.hpp>
#include <boost/graph/properties.hpp>
#include <boost/iterator/counting_iterator.hpp>
#include <boost/bind.hpp>

#include "utility.hpp"
#include <iostream>


/**
 * Laser Factor
 * @brief factor that encodes a laser measurements likelihood.
 */
class LaserFactor : public gtsam::DiscreteFactor{
  friend class OccupancyGrid;
  template <typename MultiAssignment, typename Messages>
  friend class DDLaserFactor;
  friend class BPLaserFactor;
public:
  typedef gtsam::DiscreteFactor::Values Occupancy;
  typedef typename Occupancy::mapped_type cell_value_type;
private:
	const std::vector<gtsam::Index> cells_;	///cells in which laser passes through
  const gtsam::Index factor_index_;
  const bool reflectance_;

  /** Piecewise constant function 
   * Energy = 
   *          w0_ = if (Occupancy of cells_ == w0_assignment)
   *          w1_ = if (Occupancy of cells_ == w1_assignment)
   *          w_otherwise_ = otherwise
   */
  // TODO : a mapping between w0_ and corresponding function objects ?
  const double w0_, w1_, w_otherwise_;
protected:
  
  cell_value_type w0_assignment(gtsam::Index cellidx) const {
    if (! reflectance_)
      return 0;
    else
      return (cellidx == cells_[(cells_.size() - 1)]) ? 1 : 0;
  }

  cell_value_type w1_assignment(gtsam::Index cellidx) const {
    if (! reflectance_)
      std::cout << "w1 case is undefined in case of non reflectance" << std::endl;
    return 0;
  }

  template <typename F>
  bool is_same_assignment(const F &f, const Occupancy& vals) const {
    BOOST_FOREACH(gtsam::Index c, cells_) {
      if (vals.at(c) != f(c))
        return false;
    }
    return true;
  }
  template <typename F>
  bool is_same_assignment(const F &f, const std::vector<size_t> vals) const {
    size_t  count = 0;
    BOOST_FOREACH(gtsam::Index c, cells_) {
      if (vals.at(count ++) != f(c))
        return false;
    }
    return true;
  }

public:
  typedef Occupancy argument_type;
  typedef double result_type;

	/// Constructor
	LaserFactor(const std::vector<gtsam::Index> &cells,
      const gtsam::Index idx,
      bool reflectance) : cells_(cells), factor_index_(idx), reflectance_(reflectance),
  w0_(1), w1_(900), w_otherwise_(1000) {}

  /// return node_id
  inline gtsam::Index node_id() const { return factor_index_; }

	/**
	 * Find value for given assignment of values to variables
   * returns 1000 if any of the non-last cell is occupied and 1 otherwise
   * returns 900 if the last cell is not occupied
   * returns 1 otherwise
	 */
	virtual double operator()(const Occupancy &vals) const;

	/// Multiply in a DecisionTreeFactor and return the result as DecisionTreeFactor
	virtual gtsam::DecisionTreeFactor operator*(const gtsam::DecisionTreeFactor&) const{
		throw std::runtime_error("operator * not implemented");
	}

	virtual operator gtsam::DecisionTreeFactor() const{
		throw std::runtime_error("operator DecisionTreeFactor not implemented");
	}

  virtual gtsam::DecisionTreeFactor toDecisionTreeFactor() const {
		throw std::runtime_error("toDecisionTreeFactor not implemented");
  }


};
