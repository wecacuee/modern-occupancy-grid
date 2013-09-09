// What do we all need?
//
// External datastructures (information):
// 1. Graph
//      a. Should provide an range over 
//      factors() and
//      variables()
//      b. Should comply with Boost.Graph.AdjacencyGraph, AdjacencyMatrixGraph and VertexListGraph
// 2. SlaveMinimizer should support:
//      a. get(slave_minimzer, factor)(Messages, MultiAssignment)
// 3. SampleSpaceMap should support:
//      a. for (it = get(sample_space_map, variable).first;
//              it!= get(sample_space_map, variable).second; ++it)
//
// Internal datastructures:
//
//    2. A data structure for variable assignments : MultiAssignment
//    a PropertyMap with key as undirected edge -> sample_space_element (bool or size_t)
//    3. A data structure for Lagrange dual variables: Messages
//    a PropertyMap with key as undirected edge and sample_space_element -> energy_type
//    4. A data structure for `MessageGradients`
//    a PropertyMap with key as undirected edge and sample_space_element -> energy_type
//    5. A data structure to record disagreement among factors Disagreement
// factor -> boolean
//
// Pseudo code:
// initialize() Messages with 0
// while max_iterations > 0:
//     for each factor in graph that is in Disagreement[factor]
//         minimize_slave_problem(Messages) to update MultiAssignment
//     for each variable in graph:
//         disagrees = false
//         if disagreement() in MultiAssignment:
//             resolve_disagreement() to update Messages
//             disagrees = true
//         for factors in adjacent_vertices(variable):
//             Disagreement[factor] = disagrees
//
#include <boost/graph/graph_traits.hpp>
#include <boost/foreach.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/typeof/typeof.hpp>
#include <boost/range/size.hpp>
#include <boost/type_traits.hpp>
#include <boost/unordered_map.hpp>
#include <boost/utility/result_of.hpp>
#include "debugflags.h"
#include "visualiser.h"

#include "gtsam/discrete/Assignment.h"

namespace occgrid {

struct on_disagreeing_factor { };
struct on_disagreeing_variable { };
struct on_disagreement_computation { };

template <typename Graph, typename Messages, typename MultiAssignment, typename EnergyType>
struct subgradient_update 
  : public boost::base_visitor<subgradient_update<Graph, Messages, MultiAssignment, EnergyType> >
{
  typedef typename boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
  typedef on_disagreeing_variable event_filter;
  subgradient_update(
      Messages& messages,
      MultiAssignment& multi_assignment,
      EnergyType stepsize)
    : messages_(messages),
    multi_assignment_(multi_assignment),
    stepsize_(stepsize)
  { }

  void operator()(vertex_descriptor x, const Graph& g_) {
    typedef typename boost::property_traits<Messages>::key_type msg_key_type;
    BOOST_FOREACH(vertex_descriptor f, adjacent_vertices(x, g_))
      messages_[msg_key_type(f, x, get(multi_assignment_, std::make_pair(f, x)))] += stepsize_;
  }
  private:
  Messages& messages_;
  const MultiAssignment& multi_assignment_;
  EnergyType stepsize_;
};

template <typename G, typename Minimizer, typename Messages, typename MultiAssignment>
struct laser_factor_disagreement_visitor
: public boost::base_visitor<laser_factor_disagreement_visitor<G, Minimizer, Messages, MultiAssignment> >
{
  typedef on_disagreeing_factor event_filter;
  typedef typename boost::graph_traits<G>::vertex_descriptor vertex_descriptor;
  laser_factor_disagreement_visitor(
      const Minimizer& minmizer,
      Messages& messages,
      MultiAssignment& multi_assignment) 
    : minmizer_(minmizer),
    messages_(messages),
    multi_assignment_(multi_assignment)
  { }

  void operator()(vertex_descriptor f, const G& g) {
    get(minmizer_, f)(multi_assignment_, messages_);
  }

  private:
    const Minimizer minmizer_;
    Messages& messages_;
    MultiAssignment& multi_assignment_;
};

template <typename Graph, typename MultiAssignment>
bool disagrees(
    typename boost::graph_traits<Graph>::vertex_descriptor x,
    const Graph& g,
    MultiAssignment& multi_assignment)
{
  typedef typename boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
  bool allsame = true;
  bool first_iter = true;
  vertex_descriptor f1;
  BOOST_FOREACH(vertex_descriptor f, adjacent_vertices(x, g)) {
    if (first_iter) {
      f1 = f;
      first_iter = false;
    }
    if (multi_assignment[std::make_pair(f, x)]
        != multi_assignment[std::make_pair(f1, x)]) {
      allsame = false;
      break;
    }
  }
  return (! allsame);
}

template <typename G, typename MultiAssignment>
struct DisagreementTracker {
  typedef typename MultiAssignment::key_type key_type;
  typedef typename MultiAssignment::value_type value_type;
  typedef typename MultiAssignment::reference reference;
  typedef typename boost::writable_property_map_tag category;
  typedef typename boost::graph_traits<G>::vertex_descriptor vertex_descriptor;
  DisagreementTracker(const G& g,
      MultiAssignment& multi_assignment
      ) : g_(g) , multi_assignment_(multi_assignment),
  disagreeing_vertices_(), variable_disagrement_count_(0),
  is_cache_valid_(false)
  {
    // Initialize
    BOOST_FOREACH(vertex_descriptor f, factors(g_))
      disagreeing_vertices_[f] = true;
  }

  reference get(typename MultiAssignment::key_type k) const {
    return occgrid::get(multi_assignment_, k);
  }

  void put(typename MultiAssignment::key_type k, typename MultiAssignment::value_type v) {
    occgrid::put(multi_assignment_, k, v);
    is_cache_valid_ = false;
  }

  bool is_disagreement(const vertex_descriptor& x) const {
    if (( ! is_factor(x, g_)) && (! is_cache_valid_)) {
      update_cache();
    }
    return disagreeing_vertices_[x];
  }

  size_t disagreement_count() const {
    if (! is_cache_valid_) {
      update_cache();
    }
    return variable_disagrement_count_;
  }

  private:
    void update_cache() const {
      std::cout << "Updating disagreement cache" << std::endl;
      variable_disagrement_count_ = 0;
      BOOST_FOREACH(vertex_descriptor xi, variables(g_)) {
        disagreeing_vertices_[xi] = disagrees(xi, g_, multi_assignment_);
        if (disagreeing_vertices_[xi]) {
          BOOST_FOREACH(vertex_descriptor fi, adjacent_vertices(xi, g_))
            disagreeing_vertices_[fi] = true; // any disagreement is total disagreement
          variable_disagrement_count_ += 1;
        }
      }
      is_cache_valid_ = true;
    }
    const G& g_;
    MultiAssignment& multi_assignment_;
    mutable boost::unordered_map<vertex_descriptor, bool> disagreeing_vertices_;
    mutable size_t variable_disagrement_count_;
    mutable bool is_cache_valid_;
};


template <typename DisagreementTracker>
bool is_disagreement(
    const DisagreementTracker& dt,
    typename DisagreementTracker::key_type::first_type x) 
{
  return dt.is_disagreement(x);
}

template <typename G, typename MultiAssignment>
size_t disagreement_count(const DisagreementTracker<G, MultiAssignment>& dt) {
  return dt.disagreement_count();
}

template <typename Messages, typename G, typename sample_space_type>
typename boost::property_traits<Messages>::value_type
vertex_energy(const Messages& msgs,
    const G& g,
    typename boost::graph_traits<G>::vertex_descriptor x,
    sample_space_type xv) 
{
  typedef typename boost::graph_traits<G>::vertex_descriptor vertex_descriptor;
  typedef typename boost::property_traits<Messages>::value_type energy_type;
  typedef typename boost::property_traits<Messages>::key_type msg_key_type;
  energy_type venergy(0);
  BOOST_FOREACH(vertex_descriptor f, adjacent_vertices(x, g))
    venergy += msgs[msg_key_type(f, x, xv)];
  return venergy;
}

template <typename FactorGraph, typename DualDecompositionVisitor, typename DisagreementTracker>
size_t dual_decomposition_traversal(const FactorGraph& g_,
    DualDecompositionVisitor visitors_,
    DisagreementTracker& disagreement_tracker_) 
{
  typedef typename boost::graph_traits<FactorGraph>::vertex_descriptor vertex_descriptor;
  BOOST_FOREACH(vertex_descriptor f, factors(g_)) {
    if (is_disagreement(disagreement_tracker_, f)) {
      // minimize_slave_problem(Messages) to update MultiAssignment
      invoke_visitors(visitors_, f, g_, on_disagreeing_factor());
    }
  }
  std::cout << "Factor traversal done" << std::endl;

  BOOST_FOREACH(vertex_descriptor x, variables(g_)) {
    if (is_disagreement(disagreement_tracker_, x)) {
      // update messages to resolve disagreements
      invoke_visitors(visitors_, x, g_, on_disagreeing_variable());
    }
  }
  return disagreement_count(disagreement_tracker_);
}

template <typename G, typename SampleSpaceMap, typename Messages, typename DisagreementTracker>
typename boost::property_traits<SampleSpaceMap>::value_type::first_type::value_type
assignment(
    const G& g_,
    typename boost::graph_traits<G>::vertex_descriptor x,
    const Messages& messages_,
    const SampleSpaceMap& sample_space_map_,
    const DisagreementTracker& disagreement_tracker_)
{
  typedef typename boost::graph_traits<G>::vertex_descriptor vertex_descriptor;
  typedef typename boost::property_traits<Messages>::value_type energy_type;
  typedef typename boost::property_traits<SampleSpaceMap>::value_type::first_type::value_type sample_space_type;
  sample_space_type assign = (((double)rand() / RAND_MAX) < 0.5) ? 0 : 1;
  if (is_disagreement(disagreement_tracker_, x)) {
    // maximum energy assignment
    energy_type max_energy = vertex_energy(messages_, g_, x, assign);
    BOOST_FOREACH(sample_space_type xv, get(sample_space_map_, x)) {
      energy_type xv_energy = vertex_energy(messages_, g_, x, xv);
      assign = (max_energy >= xv_energy) ?  assign : xv;
    }

  } else {
    // There is agreement, take any agreed assignment
    BOOST_FOREACH(vertex_descriptor f, adjacent_vertices(x, g_)) {
      assign = get(disagreement_tracker_, std::make_pair(f, x));
      break;
    }
  }
  return assign;
}
template <typename Graph, typename SlaveMinimizer, typename SampleSpaceMap, typename DisagreementMap, typename Messages, typename SampleSpaceType, typename EnergyType> 
struct SubgradientDualDecomposition {
  typedef typename boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
  SubgradientDualDecomposition(
      const Graph& g,
      const SlaveMinimizer& slave_minimizer,
      const SampleSpaceMap& sample_space_map,
      DisagreementMap& disagreement_tracker,
      Messages& messages,
      EnergyType init_step_size
      )
    : g_(g),
    slave_minimizer_(slave_minimizer),
    sample_space_map_(sample_space_map),
    disagreement_tracker_(disagreement_tracker),
    messages_(messages),
    init_step_size_(init_step_size),
    laser_vis_(slave_minimizer, messages_, disagreement_tracker_)
  {
  }
    
  bool operator()(size_t i) {
    EnergyType step_i_x = init_step_size_ / (i + 1);
    subgradient_update<Graph, Messages, DisagreementMap, EnergyType> 
      subg_update(messages_, disagreement_tracker_, step_i_x);

    size_t count = dual_decomposition_traversal(
        g_,
        std::make_pair(laser_vis_, subg_update),
        disagreement_tracker_);

    return (count == 0);
  }

  SampleSpaceType assignment(
      typename boost::graph_traits<Graph>::vertex_descriptor x)
  {
    return occgrid::assignment(g_, x, messages_, sample_space_map_, disagreement_tracker_);
  }

  EnergyType marginal(
      typename boost::graph_traits<Graph>::vertex_descriptor x,
      SampleSpaceType value)
  {
    if ( ! is_disagreement(disagreement_tracker_, x)) {
      BOOST_FOREACH(vertex_descriptor f, adjacent_vertices(x, g_)) {
        SampleSpaceType assign = get(disagreement_tracker_, std::make_pair(f, x));
        // Agreement. Take any assignment and decide on energy values.
        return (assign == value) ? 0 : std::numeric_limits<EnergyType>::infinity();
      }
      // Agreement but there is no factor associated. This is unexplored
      // region.
      return - std::log(0.5);
    }

    // Disgreement
    // maximum energy assignment
    EnergyType min_energy = std::numeric_limits<EnergyType>::infinity(),
    max_energy = 0;
    BOOST_FOREACH(SampleSpaceType xv, get(sample_space_map_, x)) {
      EnergyType xv_energy = vertex_energy(messages_, g_, x, xv);
      if (max_energy < xv_energy)
        max_energy = xv_energy;
      if (min_energy > xv_energy)
        min_energy = xv_energy;
    }

    EnergyType xv_energy = vertex_energy(messages_, g_, x, value);
    EnergyType retenergy = (max_energy - xv_energy);
    assert(retenergy >= 0);
    return retenergy;
  }

  private:
      const Graph& g_;
      const SlaveMinimizer& slave_minimizer_;
      const SampleSpaceMap& sample_space_map_;
      DisagreementMap& disagreement_tracker_;
      Messages messages_;
      EnergyType init_step_size_;

      laser_factor_disagreement_visitor<Graph, SlaveMinimizer, Messages, DisagreementMap> laser_vis_;
};

template <typename Graph, typename Functor, typename DisagreementMap, typename sample_space_type>
void iterate_dualdecomposition(
    const Graph& g_, 
    Functor& func,
    DisagreementMap& disagreement_tracker_,
    size_t max_iter
    )
{
  typedef typename boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
  bool converged = false;

  // main loop
  clock_t st = clock();
  for (size_t i = 0; (i < max_iter) && (not converged); ++i) {
    clock_t et = clock(); 
    converged = func(i);

    // Debugging logistics
    
    size_t count = disagreement_count(disagreement_tracker_);
    std::cout << "Variable disagreement count:" << count << std::endl;

    gtsam::Assignment<gtsam::Index> best_assign;
    std::vector<double> marg(num_variables(g_));
    BOOST_FOREACH(vertex_descriptor x, variables(g_)) {
      best_assign[x] = func.assignment(x);
      marg[x] = std::exp(-func.marginal(x, 1));
    }
    double energy = g_.g_(best_assign);
    std::cout << "<Energy>\t" << ((float)(et - st)) / CLOCKS_PER_SEC << "\t" << energy << std::endl;

    global_vis_.setMarginals(marg);
    global_vis_.show(10);

    std::vector<sample_space_type> disagreeing_cells;
    BOOST_FOREACH(vertex_descriptor x, variables(g_))
      if (is_disagreement(disagreement_tracker_, x))
        disagreeing_cells.push_back(x);
    global_vis2_.highlightCells(disagreeing_cells);
    global_vis2_.reset();
    global_vis2_.show(1);
  }
  global_vis_.show();
  global_vis_.save("/tmp/dualdecomposition.png");
}
} // namespace occgrid
