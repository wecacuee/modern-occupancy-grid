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

template <typename MultiAssignment, typename G>
void print_multiassignment(const MultiAssignment& multi_assignment,
    const G& g,
    typename boost::graph_traits<G>::vertex_descriptor x,
    bool is_factor = false) 
{
  typedef typename boost::graph_traits<G>::vertex_descriptor vertex_descriptor;
  std::cout << "v=" << x << ":";
  BOOST_AUTO(adjacent, adjacent_vertices(x, g));
  BOOST_FOREACH(vertex_descriptor f, adjacent) {
    std::cout << "f=" << f << "=" << multi_assignment[(is_factor) ? std::make_pair(x, f) : std::make_pair(f, x)] << ",";
  }
  std::cout << std::endl;
}

template <typename Messages, typename G, typename SampleSpaceMap>
void print_messages(const Messages& msgs,
    const G& g,
    typename boost::graph_traits<G>::vertex_descriptor x,
    const SampleSpaceMap& sample_space_map)
{
  typedef typename boost::graph_traits<G>::vertex_descriptor vertex_descriptor;
  typedef typename boost::property_traits<Messages>::key_type msg_key_type;
  typedef typename boost::property_traits<SampleSpaceMap>::value_type sample_space_iter_pair;
  typedef typename sample_space_iter_pair::first_type sample_space_iterator;
  typedef typename std::iterator_traits<sample_space_iterator>::value_type sample_space_type;
  std::cout << "Messages:v=" << x << ":";
  BOOST_FOREACH(sample_space_type xv, get(sample_space_map, x)) {
    BOOST_AUTO(av, adjacent_vertices(x, g));
    BOOST_FOREACH(vertex_descriptor f, av)
      std::cout << "[f=" << f << "," << x << "," << xv << "]=" << msgs[msg_key_type(f, x, xv)] << ";";
    std::cout << std::endl;
  }
}


template <typename Graph, typename Messages, typename MultiAssignment, typename SampleSpaceMap>
void resolve_disagreement(const Graph& g, 
    typename boost::graph_traits<Graph>::vertex_descriptor x,
    const MultiAssignment& multi_assignment,
    Messages& messages,
    const SampleSpaceMap& sample_space_map,
    const typename boost::property_traits<Messages>::value_type step)
{
  typedef typename boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
  typedef typename boost::property_traits<Messages>::key_type msg_key_type;
  typedef typename boost::property_traits<Messages>::value_type msg_value_type;
  typedef typename boost::property_traits<SampleSpaceMap>::value_type sample_space_iter_pair;
  typedef typename sample_space_iter_pair::first_type sample_space_iterator;
  typedef typename std::iterator_traits<sample_space_iterator>::value_type sample_space_type;

  BOOST_FOREACH(vertex_descriptor f, adjacent_vertices(x, g)) {
    messages[msg_key_type(f, x, multi_assignment[std::make_pair(f, x)])] += step;
    messages[msg_key_type(x, x, multi_assignment[std::make_pair(f, x)])] -= step;
  }

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
    typedef boost::unordered_map<vertex_descriptor, energy_type> EnergyMap;
    typedef gtsam::Assignment<sample_space_type> BestAssignment;
    typedef std::vector<energy_type> VariableEnergy;

  private:
    Messages& messages_;
    MultiAssignment& multi_assignment_;
  public:
    DualDecomposition(Messages& msgs, MultiAssignment& massign) 
      : messages_(msgs),
      multi_assignment_(massign)
      {
      }

    void init(const G& g) {
    }

    void operator()(const G& g,
        const SlaveMinimizer& slave_minimizer,
        const SampleSpaceMap& sample_space_map,
        size_t max_iter) 
    {
      Disagreement disagrees_;
      EnergyMap factor_energy_;
      BestAssignment best_assign_;
      VariableEnergy average_assign_(num_variables(g));
      // Initialize
      BOOST_FOREACH(vertex_descriptor f, factors(g))
        disagrees_[f] = true;

      BOOST_FOREACH(vertex_descriptor x, variables(g))
        best_assign_[x] = 0;

      size_t disagrement_count = 1;
      energy_type step = 500;
      // main loop
      clock_t st = clock();
      for (size_t i = 0; (i < max_iter) && (disagrement_count > 0); ++i) {

        BOOST_FOREACH(vertex_descriptor f, factors(g)) {
          if (disagrees_[f]) {
            // minimize_slave_problem(Messages) to update MultiAssignment
            factor_energy_[f] = get(slave_minimizer, f)(multi_assignment_, messages_);
            assert(! isinf(factor_energy_[f]));
          }
          disagrees_[f] = false; // reset disagreement, we will look for disagreement again
        }
        // for each variable in graph:
        disagrement_count = 0;
        std::vector<vertex_descriptor> disagreeing_cells;
        BOOST_FOREACH(vertex_descriptor x, variables(g)) {
          sample_space_type assign = (((double)rand() / RAND_MAX) < 0.5) ? 0 : 1;
          if (disagrees(g, x, multi_assignment_)) {
            disagrement_count += 1;
            disagreeing_cells.push_back(x);

            BOOST_FOREACH(vertex_descriptor f, adjacent_vertices(x, g))
              disagrees_[f] = true; // any disagreement is total disagreement

            //resolve_disagreement(g, x, multi_assignment_, messages_, sample_space_map, step / (i+1));
            energy_type step_i_x = step / (i + 1);
            BOOST_FOREACH(vertex_descriptor f, adjacent_vertices(x, g)) {
              messages_[msg_key_type(f, x, multi_assignment_[std::make_pair(f, x)])] += step_i_x;
              messages_[msg_key_type(x, x, multi_assignment_[std::make_pair(f, x)])] -= step_i_x;
            }

            // minimum energy assignment
            energy_type min_energy = messages_[msg_key_type(x, x, assign)];
            BOOST_FOREACH(sample_space_type xv, get(sample_space_map, x)) {
              energy_type xv_energy = messages_[msg_key_type(x, x, xv)];
              assign = (min_energy <= xv_energy) ?  assign : xv;
            }

          } else {
            // There is agreement, take any agreed assignment
            BOOST_FOREACH(vertex_descriptor f, adjacent_vertices(x, g)) {
              assign = multi_assignment_[std::make_pair(f, x)];
              break;
            }
          }

          best_assign_[x] = assign;
          average_assign_[x] += best_assign_[x];
        }
        clock_t et = clock(); 
        std::cout << "Variable disagreement count:" << disagrement_count << std::endl;
        double energy = g.g_(best_assign_);
        std::cout << "<Energy>\t" << ((float)(et - st)) / CLOCKS_PER_SEC << "\t" << energy << std::endl;

        global_vis_.setMarginals(best_assign_);
        global_vis_.show(1);

        global_vis2_.highlightCells(disagreeing_cells);
        global_vis2_.reset();
        global_vis2_.show(1);
      }
      //global_vis_.setMarginals(average_assign_);
      global_vis_.show();
      global_vis_.save("/tmp/dualdecomposition.png");
    }
};
