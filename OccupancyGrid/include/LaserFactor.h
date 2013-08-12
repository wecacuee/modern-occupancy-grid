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
#include <iostream>


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

  // 11*O(n) == linear with a big constant factor
  template <typename Messages, typename MultiAssignment>
    typename Messages::value_type
    operator()(const Messages& msgs,
        MultiAssignment& multi_assignment) {
      typedef typename boost::property_traits<Messages>::value_type value_type;
      typedef typename boost::property_traits<Messages>::key_type msg_key_type;
      typedef typename boost::property_traits<MultiAssignment>::value_type sample_space_type;
      // compute assignment that minimizes energy

      // TODO: check if all the messages are zero or equal then return simply
      // the maximizing assignment that is w0_assignment (because w0_ is
      // minimum among w1_ and w_otherwise_)
      bool allsame = true;
      for (size_t c = 0; c < cells_.size(); ++c) {
        if (msgs[msg_key_type(factor_index_, cell_[c], 0)]
            != msg[msg_key_type(factor_index_, cell_[c], 1)]) {
          allsame = false;
          break;
        }
      }
      if (allsame) {
        value_type w0_energy(w0_);
        for (size_t c = 0; c < cells_.size(); ++c) {
          w0_energy += msgs[msg_key_type(factor_index_, cell_[c], w0_assignment(cells_[c]))];
          multi_assignment[std::make_pair(factor_index_, cells_[c])] = w0_assignment(cells_[c]);
        }
        return w0_energy;
      }
      
      // get the minimizing assignment
      namespace bl = boost::lambda;
      std::vector<sample_space_type> other_minimizing_assign(cells_.size());
      std::vector<sample_space_type> incrementatbility(cell_.size());
      // O(n)
      for (size_t c = 0; c < cells_.size(); ++c) {
        value_type msg_0( msgs[msg_key_type(factor_index_, cell_[c], 0)]);
        value_type msg_1( msgs[msg_key_type(factor_index_, cell_[c], 1)]);
        other_minimizing_assign[c] = std::min(msg_0, msg_1);
        incrementatbility[c] = std::abs(msg_0 - msg_1);
      }

      // find smallest "2" cells with smallest increments
      std::vector<size_t> indices;
      // O(n)
      std::copy(
          boost::counting_iterator<size_t>(0),
          boost::counting_iterator<size_t>(cell_.size()),
          std::back_inserter(indices));
      // O(n)
      std::nth_element(indices.begin(), boost::next(indices.begin(), 2),
          indices.end(),
          incrementatbility[bl::_1]);

      // find an assignment that is different from w0_assignment and
      // w1_assignment
      // 4*O(n)
      if (is_same_assignment( boost::bind(&LaserFactor::w0_assignment, this, _1),
            other_minimizing_assign)
          ||
          is_same_assignment( boost::bind(&LaserFactor::w1_assignment, this, _1),
            other_minimizing_assign) 
         ) {
        other_minimizing_assign[indices[0]] = 1 - other_minimizing_assign[indices[0]];
      }
      if (is_same_assignment( boost::bind(&LaserFactor::w0_assignment, this, _1),
            other_minimizing_assign)
          ||
          is_same_assignment( boost::bind(&LaserFactor::w1_assignment, this, _1),
            other_minimizing_assign) 
         ) {
        other_minimizing_assign[indices[0]] = 1 - other_minimizing_assign[indices[0]];
        other_minimizing_assign[indices[1]] = 1 - other_minimizing_assign[indices[1]];
      }
      // compute energies corresponding to each of them
      // O(n)
      value_type w0_energy(w0_);
      for (size_t c = 0; c < cells_.size(); ++c)
        w0_energy += msgs[msg_key_type(factor_index_, cell_[c], w0_assignment(cells_[c]))];

      // O(n)
      value_type w1_energy(w1_);
      for (size_t c = 0; c < cells_.size(); ++c)
        w1_energy += msgs[msg_key_type(factor_index_, cell_[c], w1_assignment(cells_[c]))];

      // O(n)
      value_type w_otherwise_energy(w_otherwise_);
      for (size_t c = 0; c < cells_.size(); ++c)
        w_otherwise_energy += msgs[msg_key_type(factor_index_, cell_[c], other_minimizing_assign[c])];

      // Choose the case with minimum energy
      // O(n)
      value_type minimum_energy(0);
      if ((w0_energy < w1_energy) && (w0_energy < w_otherwise_energy)) {
        for (size_t c = 0; c < cells_.size(); ++c)
          multi_assignment[std::make_pair(factor_index_, cells_[c])] = w0_assignment(cells_[c]);
        minimum_energy = w0_energy;
      } else if ((w1_energy < w0_energy) && (w1_energy < w_otherwise_energy)) {
        for (size_t c = 0; c < cells_.size(); ++c)
          multi_assignment[std::make_pair(factor_index_, cells_[c])] = w1_assignment(cells_[c]);
        minimum_energy = w1_energy;
      } else {
        for (size_t c = 0; c < cells_.size(); ++c)
          multi_assignment[std::make_pair(factor_index_, cells_[c])] = other_minimizing_assign[c];
        minimum_energy = w_otherwise_energy;
      }
      return minimum_energy;
    }


  /// Return the belief of `cell` begin in state `gtsam::Value` given the
  /// Belief messages `msgs`
  template <typename MessageValues>
  typename MessageValues::value_type // can be of type logdouble/double
  belief(
      gtsam::Index cell,
      const typename Occupancy::mapped_type &cell_value,
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
      w0_prob = value_type(1);
      for (BOOST_AUTO(it, fi_begin);it != fi_end; ++it)
        w0_prob *= msgs[msg_key_type(*it, factor_index_, w0_assignment(*it))];
    }

    // Case 2
    value_type w1_prob(0);
    if (w1_assignment(cell) == cell_value) {
      w1_prob = value_type(1);
      for (BOOST_AUTO(it, fi_begin);it != fi_end; ++it)
        w1_prob *= msgs[msg_key_type(*it, factor_index_, w1_assignment(*it))];
    }
  
    value_type sum(0);
    using occgrid::toprobability;
    value_type w0_vt(toprobability<value_type>(w0_));
    value_type w1_vt( toprobability<value_type>(w1_));
    value_type w_otherwise_vt(toprobability<value_type>(w_otherwise_));
    //std::cout << "Going to print something" << std::endl;
    sum += w0_vt * w0_prob; // case 1
    //std::cout << sum << "+= " << w0_vt << " * " << w0_prob << std::endl;
    sum += w1_vt * w1_prob; // case 2
    //std::cout << sum << "+= " << w1_vt << " * " << w1_prob << std::endl;
    value_type one(1);
    sum += w_otherwise_vt * (one - w0_prob - w1_prob); // otherwise
    //std::cout << sum << "+= " << w_otherwise_vt << " * " << "(1 - " << w0_prob << " - " << w1_prob << ")" << std::endl;
    //std::cout << "w0_prob:" << w0_prob << "; w1_prob:" << w1_prob << "; w_otherwise_:" << (one - w0_prob - w1_prob) << "; sum:" << sum << std::endl;
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
