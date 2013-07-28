/**
 * @file : cartesian_product.h
 * @author : Vikas Dhiman
 * @date: Wed July 11 2013
 */

#pragma once

#include <boost/function.hpp>
#include <vector>


namespace occgrid {

/**
 * @brief: Produces a cartesian product of assignments to a Iterator of
 * variable nodes
 *
 * @type : InputIterator : A type of Iterator of VariableNodes
 * @type : PossibleValueIter : A type of Iterator of VariableNodes
 */
template<typename InputIterator, typename SampleSpaceMap>
class CartesianProduct {
private:
  typedef typename std::iterator_traits<InputIterator>::value_type Vnode;
  typedef typename SampleSpaceMap::value_type::first_type PossibleValueIter;
  typedef typename std::iterator_traits<PossibleValueIter>::value_type Value;

  // Nodes with possible assignments
	const InputIterator nodes_begin_;
  const InputIterator nodes_end_;

  const SampleSpaceMap& cdmap_;

  // A vector of value iterators pointing to the current value of each node
  std::vector< PossibleValueIter > current_val_;
  
  // last_iteration_
  bool last_iteration_;

public:
  CartesianProduct(
      InputIterator nodes_begin,
      InputIterator nodes_end,
      SampleSpaceMap cdmap)
    : nodes_begin_(nodes_begin),
    nodes_end_(nodes_end),
    cdmap_(cdmap),
    current_val_(),
    last_iteration_(false)
    {
      // Initialize all values
      for (InputIterator it = nodes_begin_; it != nodes_end_; ++it) {
        current_val_.push_back(get(cdmap_, *it).first);
      }
    }

  /// Return the next assignment
  template<typename PropertyMap>
  bool next(PropertyMap &assign) {
    bool carry_over = true;
    typedef typename std::vector< PossibleValueIter >::iterator val_it_it;
    val_it_it node_val_it_it(current_val_.begin());
    InputIterator node_it(nodes_begin_);
    for(;(node_val_it_it != current_val_.end()) 
        && (node_it != nodes_end_);
        ++ node_val_it_it, ++ node_it) 
    {
      PossibleValueIter &node_val_it = *node_val_it_it;
      const Vnode &node = *node_it;
      PossibleValueIter poss_val_end(get(cdmap_, node).second);
      Value val = *node_val_it;
      //std::cout << node << " -> " << val << std::endl;
      assign[node] = val;

      if (carry_over)
        node_val_it ++;

      if (node_val_it == poss_val_end) {
        // loop around and carry over the carry_over
        node_val_it = get(cdmap_, node).first;
        // values exhausted carry over to next loop
        carry_over = true;
      } else {
        // nothing to carry over to next loop
        carry_over = false;
      }
    }
    // If we complete the loop that means we have exhausted all possible cases
    bool is_last_iteration = last_iteration_;
    last_iteration_ = (carry_over);
    return (! is_last_iteration);
  }
};

template<typename InputIterator,
  typename SampleSpaceMap,
  typename Real,
  typename PropertyMap>
Real summaryOf(
		const boost::function<Real (PropertyMap&) > &func,
		InputIterator dependent_nodes_begin,
		InputIterator dependent_nodes_end,
    SampleSpaceMap& cdmap,
		typename std::iterator_traits<InputIterator>::value_type &x,
    typename SampleSpaceMap::value_type::first_type::value_type &xv
    )
{
  typedef typename std::iterator_traits<InputIterator>::value_type Vnode;

  CartesianProduct<InputIterator, SampleSpaceMap> poss_assign(
      dependent_nodes_begin, dependent_nodes_end, cdmap);
  Real summary(0);
  PropertyMap assign;
  while (poss_assign.next(assign)) {
    if (get(assign,x) == xv) {
      summary += func(assign);
    }
  }
  return summary;
}

} // namespace occgrid
