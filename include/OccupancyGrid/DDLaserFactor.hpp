#pragma once
#include "debugflags.h"
#include "LaserFactor.h"
template <typename MultiAssignment, typename Messages>
class DDLaserFactor {
  const LaserFactor& lf_;
  public:
  DDLaserFactor(const LaserFactor& lf) : lf_(lf) {}
  typedef MultiAssignment& arg1_type;
  typedef const Messages& arg2_type;
  typedef typename Messages::value_type result_type;

  // REquired for dual decomposition
  // 13*O(n) == linear with a big constant factor
  // This minimizes the slave problem
  typename Messages::value_type
    operator()(
        MultiAssignment& multi_assignment,
        const Messages& msgs
        ) {
      namespace bl = boost::lambda;
      typedef typename boost::property_traits<Messages>::value_type value_type;
      typedef typename boost::property_traits<Messages>::key_type msg_key_type;
      //typedef typename boost::property_traits<MultiAssignment>::value_type sample_space_type;
      typedef typename MultiAssignment::value_type sample_space_type;
      // compute assignment that minimizes energy

      // TODO: check if all the messages are zero or equal then return simply
      // the maximizing assignment that is w0_assignment (because w0_ is
      // minimum among w1_ and w_otherwise_)
      // 1.O(n)
      // bool allsame = true;
      // for (size_t c = 0; c < lf_.cells_.size(); ++c) {
      //   if (msgs[msg_key_type(lf_.factor_index_, lf_.cells_[c], 0)]
      //       != msgs[msg_key_type(lf_.factor_index_, lf_.cells_[c], 1)]) {
      //     allsame = false;
      //     break;
      //   }
      // }
      // // 2.O(n)
      // if (allsame) {
      //   value_type w0_energy(lf_.w0_);
      //   for (size_t c = 0; c < lf_.cells_.size(); ++c) {
      //     w0_energy += msgs[msg_key_type(lf_.factor_index_, lf_.cells_[c], lf_.w0_assignment(lf_.cells_[c]))];
      //     multi_assignment[std::make_pair(lf_.factor_index_, lf_.cells_[c])] = lf_.w0_assignment(lf_.cells_[c]);
      //   }
      //   return w0_energy;
      // }

      // get the minimizing assignment
      std::vector<sample_space_type> other_minimizing_assign(lf_.cells_.size());
      std::vector<sample_space_type> incrementatbility(lf_.cells_.size());
      // 3.O(n)
      // std::cout << "msg values:";
      for (size_t c = 0; c < lf_.cells_.size(); ++c) {
        value_type msg_0( msgs[msg_key_type(lf_.factor_index_, lf_.cells_[c], 0)]);
        value_type msg_1( msgs[msg_key_type(lf_.factor_index_, lf_.cells_[c], 1)]);
        // std::cout << "(" << msg_0 << ", " << msg_1 << ")";
        other_minimizing_assign[c] = (msg_0 <= msg_1) ? 0 : 1;
        incrementatbility[c] = std::abs(msg_0 - msg_1);
      }
      // std::cout << std::endl;

      // find smallest "2" cells with smallest increments
      std::vector<size_t> indices;
      // 4.O(n)
      std::copy(
          boost::counting_iterator<size_t>(0),
          boost::counting_iterator<size_t>(lf_.cells_.size()),
          std::back_inserter(indices));
      assert(indices.size() == lf_.cells_.size());
      // 5.O(n)
      std::nth_element(indices.begin(), boost::next(indices.begin(), 2),
          indices.end(),
          (bl::var(incrementatbility)[bl::_1] < bl::var(incrementatbility)[bl::_2]));

      // find an assignment that is different from w0_assignment and
      // w1_assignment
      // [6-9]*O(n)
      if (lf_.is_same_assignment( boost::bind(&LaserFactor::w0_assignment, lf_, _1),
            other_minimizing_assign)
          ||
          (lf_.reflectance_ && lf_.is_same_assignment( boost::bind(&LaserFactor::w1_assignment, lf_, _1),
            other_minimizing_assign) )
         ) {
        other_minimizing_assign[indices[0]] = 1 - other_minimizing_assign[indices[0]];
      }
      if (lf_.is_same_assignment( boost::bind(&LaserFactor::w0_assignment, lf_, _1),
            other_minimizing_assign)
          ||
          (lf_.reflectance_ && lf_.is_same_assignment( boost::bind(&LaserFactor::w1_assignment, lf_, _1),
            other_minimizing_assign))
         ) {
        other_minimizing_assign[indices[0]] = 1 - other_minimizing_assign[indices[0]];
        other_minimizing_assign[indices[1]] = 1 - other_minimizing_assign[indices[1]];
      }
      // compute energies corresponding to each of them
      // 10.O(n)
      if (DEBUG_DD) {
        std::cout << "Message contribution f=" << lf_.factor_index_ << ":";
        for (size_t c = 0; c < lf_.cells_.size(); ++c)
          std::cout << msgs[msg_key_type(lf_.factor_index_, lf_.cells_[c], lf_.w0_assignment(lf_.cells_[c]))] <<",";
        std::cout << std::endl;
      }

      value_type w0_energy(lf_.w0_);
      for (size_t c = 0; c < lf_.cells_.size(); ++c)
        w0_energy += msgs[msg_key_type(lf_.factor_index_, lf_.cells_[c], lf_.w0_assignment(lf_.cells_[c]))];

      // 11.O(n)
      value_type w1_energy(lf_.w1_);
      if (lf_.reflectance_) {
        for (size_t c = 0; c < lf_.cells_.size(); ++c)
          w1_energy += msgs[msg_key_type(lf_.factor_index_, lf_.cells_[c], lf_.w1_assignment(lf_.cells_[c]))];
      } else {
        w1_energy = w0_energy;
      }

      // 12.O(n)
      value_type w_otherwise_energy(lf_.w_otherwise_);
      for (size_t c = 0; c < lf_.cells_.size(); ++c)
        w_otherwise_energy += msgs[msg_key_type(lf_.factor_index_, lf_.cells_[c], other_minimizing_assign[c])];

      // Choose the case with minimum energy
      // 13.O(n)
      value_type minimum_energy(0);
      value_type maximum_energy(0);
      using boost::put;
      if ((w0_energy <= w1_energy) && (w0_energy <= w_otherwise_energy)) {
        for (size_t c = 0; c < lf_.cells_.size(); ++c)
          put(multi_assignment, std::make_pair(lf_.factor_index_, lf_.cells_[c]), lf_.w0_assignment(lf_.cells_[c]));
        minimum_energy = w0_energy;
        maximum_energy = std::max(w1_energy, w_otherwise_energy);
      } else if ((w1_energy <= w0_energy) && (w1_energy <= w_otherwise_energy)) {
        for (size_t c = 0; c < lf_.cells_.size(); ++c)
          put(multi_assignment, std::make_pair(lf_.factor_index_, lf_.cells_[c]), lf_.w1_assignment(lf_.cells_[c]));
        minimum_energy = w1_energy;
        maximum_energy = std::max(w0_energy, w_otherwise_energy);
      } else {
        for (size_t c = 0; c < lf_.cells_.size(); ++c)
          put(multi_assignment, std::make_pair(lf_.factor_index_, lf_.cells_[c]), other_minimizing_assign[c]);
        minimum_energy = w_otherwise_energy;
        maximum_energy = std::max(w0_energy, w1_energy);
      }
      (void) maximum_energy;
      //std::cout << "Energy gap" << maximum_energy - minimum_energy << std::endl;
      return minimum_energy;
    }
};
