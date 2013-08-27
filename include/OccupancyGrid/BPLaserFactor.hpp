#pragma once
#include "LaserFactor.h"

class BPLaserFactor {
  private:
    const LaserFactor& lf_;
  public:
    BPLaserFactor(const LaserFactor& lf) : lf_(lf) { }
  typedef LaserFactor::argument_type argument_type;
  typedef LaserFactor::result_type result_type;

  /// Required only for belief propagation / sumproduct algorithm
  /// Return the belief of `cell` begin in state `gtsam::Value` given the
  /// Belief messages `msgs`
  template <typename MessageValues>
  typename MessageValues::value_type // can be of type logdouble/double
  belief(
      gtsam::Index cell,
      const typename LaserFactor::Occupancy::mapped_type &cell_value,
      MessageValues msgs) const 
  {
    typedef typename MessageValues::key_type msg_key_type;
    typedef typename MessageValues::value_type value_type;
    namespace bl = boost::lambda;
    BOOST_AUTO(fi_begin,
        boost::make_filter_iterator(
          (cell != bl::_1), lf_.cells_.begin(), lf_.cells_.end()));
    BOOST_AUTO(fi_end,
        boost::make_filter_iterator(
          (cell != bl::_1), lf_.cells_.end(), lf_.cells_.end()));

    // Case 1
    // this cell has deterministic probability
    value_type w0_prob(0);
    if (lf_.w0_assignment(cell) == cell_value) {
      w0_prob = value_type(1);
      for (BOOST_AUTO(it, fi_begin);it != fi_end; ++it)
        w0_prob *= msgs[msg_key_type(*it, lf_.factor_index_, lf_.w0_assignment(*it))];
    }

    // Case 2
    value_type w1_prob(0);
    if (lf_.w1_assignment(cell) == cell_value) {
      w1_prob = value_type(1);
      for (BOOST_AUTO(it, fi_begin);it != fi_end; ++it)
        w1_prob *= msgs[msg_key_type(*it, lf_.factor_index_, lf_.w1_assignment(*it))];
    }
  
    value_type sum(0);
    using occgrid::toprobability;
    value_type w0_vt(toprobability<value_type>(lf_.w0_));
    value_type w1_vt( toprobability<value_type>(lf_.w1_));
    value_type w_otherwise_vt(toprobability<value_type>(lf_.w_otherwise_));
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
};
