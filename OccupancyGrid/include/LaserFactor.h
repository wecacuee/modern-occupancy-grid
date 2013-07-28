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

#include "utility.hpp"


/**
 * Laser Factor
 * @brief factor that encodes a laser measurements likelihood.
 */
class LaserFactor : public gtsam::DiscreteFactor{
  friend class OccupancyGrid;

public:
  typedef gtsam::DiscreteFactor::Values Occupancy;
  typedef typename Occupancy::mapped_type cell_value_type;
private:
	std::vector<gtsam::Index> cells_;	///cells in which laser passes through
  const gtsam::Index factor_index_;

  /** Piecewise constant function 
   * Energy = 
   *          w0_ = if (Occupancy of cells_ == w0_assignment)
   *          w1_ = if (Occupancy of cells_ == w1_assignment)
   *          w_otherwise_ = otherwise
   */
  // TODO : a mapping between w0_ and corresponding function objects ?
  const double w0_, w1_, w_otherwise_;
  
  cell_value_type w0_assignment(gtsam::Index cellidx) const {
    return (cellidx == cells_[(cells_.size() - 1)]) ? 1 : 0;
  }

  cell_value_type w1_assignment(gtsam::Index cellidx) const {
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

public:
  typedef Occupancy argument_type;
  typedef double result_type;

	/// Constructor
	LaserFactor(const std::vector<gtsam::Index> &cells,
      const gtsam::Index idx) : cells_(cells), factor_index_(idx),
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

  /// Return the belief of `cell` begin in state `gtsam::Value` given the
  /// Belief messages `msgs`
  template <typename MessageValues>
  typename MessageValues::value_type // can be of type logdouble/double
  belief(
      gtsam::Index cell,
      typename Occupancy::mapped_type cell_value,
      MessageValues msgs) const 
  {
    typedef typename MessageValues::key_type msg_key_type;
    typedef typename MessageValues::value_type value_type;
    namespace bl = boost::lambda;
    BOOST_AUTO(fi_begin,
        boost::make_filter_iterator(
          (cell != bl::_1), cells_.begin(), cells_.end()));
    BOOST_AUTO(fi_end,
        boost::make_filter_iterator(
          (cell != bl::_1), cells_.end(), cells_.end()));

    // Case 1
    // this cell has deterministic probability
    value_type w0_prob(0);
    if (w0_assignment(cell) == cell_value) {
      w0_prob = 1;
      for (BOOST_AUTO(it, fi_begin);it != fi_end; ++it)
        w0_prob *= msgs[msg_key_type(*it, factor_index_, w0_assignment(*it))];
    }

    // Case 2
    value_type w1_prob(0);
    if (w1_assignment(cell) == cell_value) {
      w1_prob = 1;
      for (BOOST_AUTO(it, fi_begin);it != fi_end; ++it)
        w1_prob *= msgs[msg_key_type(*it, factor_index_, w1_assignment(*it))];
    }

  
    value_type sum(0);
    using occgrid::toprobability;
    sum += toprobability<value_type>(w0_) * w0_prob; // case 1
    sum += toprobability<value_type>(-w1_) * w1_prob; // case 2
    sum += toprobability<value_type>(-w_otherwise_) * (1 - w0_prob - w1_prob); // otherwise
    return sum;
  }

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
