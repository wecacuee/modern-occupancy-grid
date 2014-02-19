#include "OccupancyGrid/dualdecomposition.hpp"
#include "boost/graph/adjacency_list.hpp"

using namespace occgrid;

/// Adjacency list graph for FactorGraph
typedef boost::adjacency_list<
  boost::vecS, boost::vecS, boost::undirectedS> FactorGraph;

/// Vertex type
typedef typename boost::graph_traits< FactorGraph >::vertex_descriptor Vertex;

enum variables { x1, x2, x3, x4, x5 };
enum factors { fa = x5 + 1, fb, fc, fd, fe };

std::string vertex_name(Vertex v) {
  std::stringstream ss;
  if (v >= fa) {
    ss << "f" << static_cast<char>(v - fa + 'a');
    return ss.str();
  } else {
    ss << "x" << (v + 1);
    return ss.str();
  }
}

typedef std::size_t sample_space_type;
typedef typename std::vector<sample_space_type>::const_iterator sample_space_iterator;
typedef std::pair<sample_space_iterator, sample_space_iterator> sample_space_iter_pair;

/// Value assignment to variable nodes
typedef associative_property_map< boost::unordered_map<Vertex, sample_space_type > > AssignmentMap;

typedef boost::associative_property_map< boost::unordered_map<Vertex, bool > > IsFactorMap;

typedef boost::function<double (const AssignmentMap&)> FactorType;
typedef boost::associative_property_map< boost::unordered_map<Vertex, FactorType > > FactorMap;

double funca(const AssignmentMap& ass) {
  if (ass[x] == 0) {
    return 100;
  } else {
    return 90;
  }
}
double funcb(const AssignmentMap& ass) {
  if (ass[x] == 0) {
    return 90;
  } else {
    return 100;
  }
}

int main(int argc, const char *argv[])
{
  //
  //  |fa|   |fb|
  //     \    /
  //      \  /
  //        x
  typedef std::pair<int, int> Edge;
  Edge edge_array[] = { Edge(fa, x1), Edge(fb, x1) }
  FactorGraph g(edge_array,
      edge_array + (sizeof(edge_array) / sizeof(Edge)),
       3 );
  boost::unordered_map<Vertex, bool> is_factor_map;
  IsFactorMap is_factor(is_factor_map);
  BOOST_FOREACH(Vertex v, vertices(g)) {
    put(is_factor, v, (v < fa) ? false : true);
  }
  boost::unordered_map<Vertex, FactorType> factors_map;
  FactorMap slave_minimizer(factors_map);
  put(fmap, fa, FactorType(funca));
  put(fmap, fb, FactorType(funcb));

  return 0;
}
