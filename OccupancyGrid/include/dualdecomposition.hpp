// What do we all need?
//
// External datastructures:
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
#include "visualiser.h"

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
  }
  BOOST_AUTO(sample_space_size, boost::size(get(sample_space_map, x)));

  BOOST_FOREACH(sample_space_type xv, get(sample_space_map, x)) {
    // normalize so that messages sum up to one
    msg_value_type avg(0);
    BOOST_FOREACH(vertex_descriptor f, adjacent_vertices(x, g)) {
      avg += messages[msg_key_type(f, x, xv)];
    }
    avg /= sample_space_size;
    BOOST_FOREACH(vertex_descriptor f, adjacent_vertices(x, g)) {
      messages[msg_key_type(f, x, xv)] -= avg;
    }
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
  public:
    typedef typename boost::property_traits<Messages>::value_type energy_type;
    typedef boost::unordered_map<vertex_descriptor, energy_type> EnergyMap;
    typedef std::vector<sample_space_type> BestAssignment;

  private:
    Messages& messages_;
    MultiAssignment& multi_assignment_;
    Disagreement disagrees_;
    EnergyMap vertex_energy_;
    BestAssignment best_assign_;
  public:
    DualDecomposition(Messages& msgs, MultiAssignment& massign) 
      : messages_(msgs),
      multi_assignment_(massign),
      disagrees_(),
      vertex_energy_(),
      best_assign_() {
      }
    void init(const G& g) {
      BOOST_FOREACH(vertex_descriptor f, factors(g)) {
        disagrees_[f] = true;
      }
      best_assign_.resize(num_variables(g));
      BOOST_FOREACH(vertex_descriptor x, variables(g)) {
        vertex_energy_[x] = 1000;
        best_assign_[x] = ((((double)rand()) / RAND_MAX) < 0.5) ? 0 : 1;
      }
    }
    void operator()(const G& g,
        const SlaveMinimizer& slave_minimizer,
        const SampleSpaceMap& sample_space_map,
        size_t max_iter) 
    {
      init(g);
      size_t disagrement_count = 1;
      energy_type step = 10;
      for (size_t i = 0; (i < max_iter) && (disagrement_count > 0); ++i) {
        clock_t st = clock(); 
        size_t fact_disagreement = 0;
        BOOST_FOREACH(vertex_descriptor f, factors(g)) {
          if (disagrees_[f]) {
            // minimize_slave_problem(Messages) to update MultiAssignment
            vertex_energy_[f] = get(slave_minimizer, f)(multi_assignment_, messages_);
            assert(! isinf(vertex_energy_[f]));
            ++fact_disagreement;
          }
        }
        std::cout << "Factor disagreement count:" << fact_disagreement << std::endl;
        // for each variable in graph:
        disagrement_count = 0;
        BOOST_FOREACH(vertex_descriptor x, variables(g)) {
          bool disagreement = false;
          if (disagrees(g, x, multi_assignment_)) {
            // std::cout << "Disagreement in x:" << x << "=" ;
            // BOOST_FOREACH(vertex_descriptor f, adjacent_vertices(x, g)) {
            //   std::cout << multi_assignment_[std::make_pair(f, x)] << ", ";
            // }
            // std::cout << std::endl;

            disagreement = true;
            disagrement_count += 1;
            resolve_disagreement(g, x, multi_assignment_, messages_, sample_space_map, step / (i+1));
          }
          BOOST_FOREACH(vertex_descriptor f, adjacent_vertices(x, g)) {
            disagrees_[f] &= disagreement;
            assert(! isinf(vertex_energy_[f]));
            energy_type xe = vertex_energy_[x];
            energy_type fe = vertex_energy_[f];
            vertex_energy_[x] = (fe < xe) ? fe : xe;
            best_assign_[x] = (fe < xe) ?  multi_assignment_[std::make_pair(f, x)] : best_assign_[x];
          }
        }
        clock_t et = clock(); std::cout << " clicks taken: " << ((float)(et - st)) / CLOCKS_PER_SEC << std::endl;
        std::cout << "Variable disagreement count:" << disagrement_count << std::endl;
        global_vis_.setMarginals(best_assign_);
        global_vis_.show();
      }
    }
    energy_type energy(vertex_descriptor x) {
      return vertex_energy_[x];
    }
    sample_space_type assignment(vertex_descriptor x) {
      return best_assign_[x];
    }
};
