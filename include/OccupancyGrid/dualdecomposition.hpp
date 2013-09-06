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
#include <boost/typeof/typeof.hpp>
#include <boost/range/size.hpp>
#include <boost/type_traits.hpp>
#include <boost/unordered_map.hpp>
#include <boost/utility/result_of.hpp>
#include "debugflags.h"
#include "visualiser.h"

#include "gtsam/discrete/Assignment.h"

template <typename Graph, typename MultiAssignment>
bool disagrees(const Graph& g,
    typename boost::graph_traits<Graph>::vertex_descriptor x,
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

template <typename G, typename SlaveMinimizer, typename SampleSpaceMap, typename Messages, typename MultiAssignment>
class DualDecomposition {
  private:
    typedef typename boost::graph_traits<G>::vertex_descriptor vertex_descriptor;
    typedef typename boost::property_traits<SlaveMinimizer>::value_type functor_type;
    typedef typename boost::property_traits<SampleSpaceMap>::value_type sample_space_iter_pair;
    typedef typename sample_space_iter_pair::first_type sample_space_iterator;
    typedef typename std::iterator_traits<sample_space_iterator>::value_type sample_space_type;
    typedef typename boost::graph_traits<G>::edge_descriptor edge_descriptor;
    struct message_key_type : public std::pair<edge_descriptor, sample_space_type> {
      message_key_type(edge_descriptor e, sample_space_type xv) 
        : std::pair<edge_descriptor, sample_space_type>(e, xv) {}
    };
    typedef boost::unordered_map<vertex_descriptor, bool> Disagreement;
    typedef typename boost::property_traits<Messages>::key_type msg_key_type;
  public:
    typedef typename boost::property_traits<Messages>::value_type energy_type;

  private:
    const G& g_;
    const SlaveMinimizer& slave_minimizer_;
    const SampleSpaceMap& sample_space_map_;
    Messages& messages_;
    MultiAssignment& multi_assignment_;
    energy_type step_size_;
    Disagreement disagreeing_vertices_;
  public:
    DualDecomposition(
        const G& g,
        const SlaveMinimizer& slave_minimizer,
        const SampleSpaceMap& sample_space_map,
        Messages& msgs, 
        MultiAssignment& massign,
        energy_type step) 
      : g_(g),
      slave_minimizer_(slave_minimizer),
      sample_space_map_(sample_space_map),
      messages_(msgs),
      multi_assignment_(massign),
      step_size_(step),
      disagreeing_vertices_()
      {
      }

    void minimize_slaves() {
        BOOST_FOREACH(vertex_descriptor f, factors(g_)) {
          if (disagreeing_vertices_[f]) {
            // minimize_slave_problem(Messages) to update MultiAssignment
            get(slave_minimizer_, f)(multi_assignment_, messages_);
          }
          disagreeing_vertices_[f] = false; // reset disagreement, we will look for disagreement again
        }
    }

    size_t resolve_disagreements(energy_type stepsize) {
      // for each variable in graph:
      size_t disagrement_count = 0;
      BOOST_FOREACH(vertex_descriptor x, variables(g_)) {
        disagreeing_vertices_[x] = false;
        if (disagrees(g_, x, multi_assignment_)) {
          // update messages to resolve disagreements
          //get(slave_minimizer_, x)(multi_assignment_, messages_);
          BOOST_FOREACH(vertex_descriptor f, adjacent_vertices(x, g_))
            messages_[msg_key_type(f, x, multi_assignment_[std::make_pair(f, x)])] += stepsize;

          // logistics
          disagrement_count += 1;
          disagreeing_vertices_[x] = true;
          BOOST_FOREACH(vertex_descriptor f, adjacent_vertices(x, g_))
            disagreeing_vertices_[f] = true; // any disagreement is total disagreement
        }
      }
      return disagrement_count;
    }

    sample_space_type assignment(vertex_descriptor x) {
      sample_space_type assign = (((double)rand() / RAND_MAX) < 0.5) ? 0 : 1;
      if (disagreeing_vertices_[x]) {
        // maximum energy assignment
        energy_type max_energy = vertex_energy(messages_, g_, x, assign);
        BOOST_FOREACH(sample_space_type xv, get(sample_space_map_, x)) {
          energy_type xv_energy = vertex_energy(messages_, g_, x, xv);
          assign = (max_energy >= xv_energy) ?  assign : xv;
        }

      } else {
        // There is agreement, take any agreed assignment
        BOOST_FOREACH(vertex_descriptor f, adjacent_vertices(x, g_)) {
          assign = multi_assignment_[std::make_pair(f, x)];
          break;
        }
      }
      return assign;
    }

    void operator()(size_t max_iter) {
      // Initialize
      BOOST_FOREACH(vertex_descriptor f, factors(g_))
        disagreeing_vertices_[f] = true;

      size_t disagrement_count = 1;
      // main loop
      clock_t st = clock();
      for (size_t i = 0; (i < max_iter) && (disagrement_count > 0); ++i) {
        clock_t et = clock(); 
        // slave minimization
        minimize_slaves();
        // message update for disagreement resolution
        energy_type step_i_x = step_size_ / (i + 1);
        disagrement_count = resolve_disagreements(step_i_x);

        // Debugging logistics
        gtsam::Assignment<gtsam::Index> best_assign;
        BOOST_FOREACH(vertex_descriptor x, variables(g_))
          best_assign[x] = assignment(x);
        std::cout << "Variable disagreement count:" << disagrement_count << std::endl;
        double energy = g_.g_(best_assign);
        std::cout << "<Energy>\t" << ((float)(et - st)) / CLOCKS_PER_SEC << "\t" << energy << std::endl;

        global_vis_.setMarginals(best_assign);
        global_vis_.show(1);

        std::vector<sample_space_type> disagreeing_cells;
        BOOST_FOREACH(vertex_descriptor x, variables(g_))
          if (disagreeing_vertices_[x])
            disagreeing_cells.push_back(x);
        global_vis2_.highlightCells(disagreeing_cells);
        global_vis2_.reset();
        global_vis2_.show(1);
      }
      global_vis_.show();
      global_vis_.save("/tmp/dualdecomposition.png");
    }

};
