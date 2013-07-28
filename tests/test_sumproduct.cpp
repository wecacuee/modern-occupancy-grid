#include "sumproduct.hpp"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/lexical_cast.hpp>
#include <gtest/gtest.h>
#include "utility.hpp"
using namespace occgrid;

/// Adjacency list graph for FactorGraph
typedef boost::adjacency_list<
  boost::vecS, boost::vecS, boost::undirectedS> FactorGraph;


/// Vertex type
typedef typename boost::graph_traits< FactorGraph >::vertex_descriptor Vertex;

typedef std::size_t sample_space_type;
typedef typename std::vector<sample_space_type>::const_iterator codomain_iterator;
typedef std::pair<codomain_iterator, codomain_iterator> codomain_iter_pair;
/// Codomain functor
//
codomain_iter_pair codomain_static_function() {
  const static std::size_t codomain_vector[] = {0, 1};
  return std::make_pair(codomain_vector, codomain_vector + 2);
}
struct codomain_func : public std::unary_function<Vertex,  codomain_iter_pair> {
  codomain_iter_pair operator()(const Vertex& v) const {
    const static std::size_t codomain_vector[] = {0, 1};
    return std::make_pair(codomain_vector, codomain_vector + 2);
  }
};

/// Codomain Property Map for variable nodes
class BinaryCodomainMap {
  public:
  typedef Vertex key_type;
  typedef codomain_iter_pair value_type;
  typedef value_type& reference;
  typedef boost::lvalue_property_map_tag category;
};
BinaryCodomainMap::value_type get(BinaryCodomainMap c, Vertex n) {
  const static std::size_t codomain_array[] = {0, 1};
  const static std::vector<sample_space_type> codomain_vector(
      codomain_array, codomain_array + 2);
  return std::make_pair(codomain_vector.begin(), codomain_vector.end());
}

//=========================================================================
// An adaptor to turn a Unique Pair Associative Container like std::map or
// std::hash_map into an Lvalue Property Map.

template <typename UniquePairAssociativeContainer>
class associative_property_map
  : public boost::put_get_helper<
     typename UniquePairAssociativeContainer::value_type::second_type&,
     associative_property_map<UniquePairAssociativeContainer> >
{
  typedef UniquePairAssociativeContainer C;
public:
  typedef typename C::key_type key_type;
  typedef typename C::value_type::second_type value_type;
  typedef value_type& reference;
  typedef boost::lvalue_property_map_tag category;
  associative_property_map() : m_c() { }
  reference operator[](const key_type& k) const {
    return (const_cast<C&>(m_c))[k];
  }
private:
  C m_c;
};
//=========================================================================

/// Value assignment to variable nodes
typedef associative_property_map< boost::unordered_map<Vertex, sample_space_type > > AssignmentMap;

/// Factor map for factor nodes
typedef boost::function<SymReal (const AssignmentMap&)> FactorType;
typedef boost::associative_property_map< boost::unordered_map<Vertex, FactorType > > FactorMap;

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

template<typename VertexIterator>
struct vertex_factor {
private:
  typename VertexIterator::value_type v_;
  VertexIterator v_begin_;
  VertexIterator v_end_;
public:
  vertex_factor(typename VertexIterator::value_type v, VertexIterator v_begin, VertexIterator v_end) 
    : v_(v), v_begin_(v_begin), v_end_(v_end) { 
      // std::cout << "Neigbours of " << v_ << " are:";
      // for (VertexIterator nbr(v_begin_); nbr != v_end_; ++nbr) std::cout << *nbr << "," ;
      // std::cout << std::endl;
    }

  SymReal operator()(const AssignmentMap& amap) const {
    std::stringstream ss;
    ss << vertex_name(v_) << "(";
    for (VertexIterator v(v_begin_); v != v_end_; ++v) 
      ss << vertex_name(*v) << "=" << amap[*v] << ", ";
    ss << ")";
    return SymReal(ss.str());
  }
};

struct MessageKeyType : public std::pair< std::pair<Vertex, Vertex>, sample_space_type> {
  explicit MessageKeyType(Vertex u, Vertex v, sample_space_type ss) : 
    std::pair< std::pair<Vertex, Vertex>, sample_space_type>(std::pair<Vertex, Vertex>(u, v), ss) {};
};
typedef boost::unordered_map<MessageKeyType, SymReal > MessageValuesBaseType;
typedef boost::associative_property_map<MessageValuesBaseType> MessageValues;
typedef boost::associative_property_map< boost::unordered_map<Vertex, bool > > IsFactorMap;

const std::string* expected_output() {
  const static std::string strarray[] = 
  {
    "mu(x3->fc:1) = [fd(x3=1, x4=0, ) + fd(x3=1, x4=1, )][fe(x3=1, x5=0, ) + fe(x3=1, x5=1, )]",
"mu(x4->fd:1) = 1",
"mu(x4->fd:0) = 1",
"mu(fa->x1:1) = fa(x1=1, )",
"mu(fc->x1:1) = [  fc(x3=0, x2=0, x1=1, )[fd(x3=0, x4=0, ) + fd(x3=0, x4=1, )][fe(x3=0, x5=0, ) + fe(x3=0, x5=1, )]fb(x2=0, ) + fc(x3=1, x2=0, x1=1, )[fd(x3=1, x4=0, ) + fd(x3=1, x4=1, )][fe(x3=1, x5=0, ) + fe(x3=1, x5=1, )]fb(x2=0, )  + fc(x3=0, x2=1, x1=1, )[fd(x3=0, x4=0, ) + fd(x3=0, x4=1, )][fe(x3=0, x5=0, ) + fe(x3=0, x5=1, )]fb(x2=1, )  + fc(x3=1, x2=1, x1=1, )[fd(x3=1, x4=0, ) + fd(x3=1, x4=1, )][fe(x3=1, x5=0, ) + fe(x3=1, x5=1, )]fb(x2=1, )]",
"mu(fa->x1:0) = fa(x1=0, )",
"mu(fc->x1:0) = [  fc(x3=0, x2=0, x1=0, )[fd(x3=0, x4=0, ) + fd(x3=0, x4=1, )][fe(x3=0, x5=0, ) + fe(x3=0, x5=1, )]fb(x2=0, ) + fc(x3=1, x2=0, x1=0, )[fd(x3=1, x4=0, ) + fd(x3=1, x4=1, )][fe(x3=1, x5=0, ) + fe(x3=1, x5=1, )]fb(x2=0, )  + fc(x3=0, x2=1, x1=0, )[fd(x3=0, x4=0, ) + fd(x3=0, x4=1, )][fe(x3=0, x5=0, ) + fe(x3=0, x5=1, )]fb(x2=1, )  + fc(x3=1, x2=1, x1=0, )[fd(x3=1, x4=0, ) + fd(x3=1, x4=1, )][fe(x3=1, x5=0, ) + fe(x3=1, x5=1, )]fb(x2=1, )]",
"mu(x5->fe:1) = 1",
"mu(x5->fe:0) = 1",
"mu(fc->x2:1) = [  fc(x3=0, x2=1, x1=0, )[fd(x3=0, x4=0, ) + fd(x3=0, x4=1, )][fe(x3=0, x5=0, ) + fe(x3=0, x5=1, )]fa(x1=0, ) + fc(x3=1, x2=1, x1=0, )[fd(x3=1, x4=0, ) + fd(x3=1, x4=1, )][fe(x3=1, x5=0, ) + fe(x3=1, x5=1, )]fa(x1=0, )  + fc(x3=0, x2=1, x1=1, )[fd(x3=0, x4=0, ) + fd(x3=0, x4=1, )][fe(x3=0, x5=0, ) + fe(x3=0, x5=1, )]fa(x1=1, )  + fc(x3=1, x2=1, x1=1, )[fd(x3=1, x4=0, ) + fd(x3=1, x4=1, )][fe(x3=1, x5=0, ) + fe(x3=1, x5=1, )]fa(x1=1, )]",
"mu(fc->x2:0) = [  fc(x3=0, x2=0, x1=0, )[fd(x3=0, x4=0, ) + fd(x3=0, x4=1, )][fe(x3=0, x5=0, ) + fe(x3=0, x5=1, )]fa(x1=0, ) + fc(x3=1, x2=0, x1=0, )[fd(x3=1, x4=0, ) + fd(x3=1, x4=1, )][fe(x3=1, x5=0, ) + fe(x3=1, x5=1, )]fa(x1=0, )  + fc(x3=0, x2=0, x1=1, )[fd(x3=0, x4=0, ) + fd(x3=0, x4=1, )][fe(x3=0, x5=0, ) + fe(x3=0, x5=1, )]fa(x1=1, )  + fc(x3=1, x2=0, x1=1, )[fd(x3=1, x4=0, ) + fd(x3=1, x4=1, )][fe(x3=1, x5=0, ) + fe(x3=1, x5=1, )]fa(x1=1, )]",
"mu(x1->fc:0) = fa(x1=0, )",
"mu(x1->fc:1) = fa(x1=1, )",
"mu(fd->x3:1) = [fd(x3=1, x4=0, ) + fd(x3=1, x4=1, )]",
"mu(fd->x3:0) = [fd(x3=0, x4=0, ) + fd(x3=0, x4=1, )]",
"mu(x1->fa:0) = [  fc(x3=0, x2=0, x1=0, )[fd(x3=0, x4=0, ) + fd(x3=0, x4=1, )][fe(x3=0, x5=0, ) + fe(x3=0, x5=1, )]fb(x2=0, ) + fc(x3=1, x2=0, x1=0, )[fd(x3=1, x4=0, ) + fd(x3=1, x4=1, )][fe(x3=1, x5=0, ) + fe(x3=1, x5=1, )]fb(x2=0, )  + fc(x3=0, x2=1, x1=0, )[fd(x3=0, x4=0, ) + fd(x3=0, x4=1, )][fe(x3=0, x5=0, ) + fe(x3=0, x5=1, )]fb(x2=1, )  + fc(x3=1, x2=1, x1=0, )[fd(x3=1, x4=0, ) + fd(x3=1, x4=1, )][fe(x3=1, x5=0, ) + fe(x3=1, x5=1, )]fb(x2=1, )]",
"mu(x1->fa:1) = [  fc(x3=0, x2=0, x1=1, )[fd(x3=0, x4=0, ) + fd(x3=0, x4=1, )][fe(x3=0, x5=0, ) + fe(x3=0, x5=1, )]fb(x2=0, ) + fc(x3=1, x2=0, x1=1, )[fd(x3=1, x4=0, ) + fd(x3=1, x4=1, )][fe(x3=1, x5=0, ) + fe(x3=1, x5=1, )]fb(x2=0, )  + fc(x3=0, x2=1, x1=1, )[fd(x3=0, x4=0, ) + fd(x3=0, x4=1, )][fe(x3=0, x5=0, ) + fe(x3=0, x5=1, )]fb(x2=1, )  + fc(x3=1, x2=1, x1=1, )[fd(x3=1, x4=0, ) + fd(x3=1, x4=1, )][fe(x3=1, x5=0, ) + fe(x3=1, x5=1, )]fb(x2=1, )]",
"mu(fe->x5:0) = [fe(x3=0, x5=0, )[  fc(x3=0, x2=0, x1=0, )fb(x2=0, )fa(x1=0, ) + fc(x3=0, x2=1, x1=0, )fb(x2=1, )fa(x1=0, )  + fc(x3=0, x2=0, x1=1, )fb(x2=0, )fa(x1=1, )  + fc(x3=0, x2=1, x1=1, )fb(x2=1, )fa(x1=1, )][fd(x3=0, x4=0, ) + fd(x3=0, x4=1, )] + fe(x3=1, x5=0, )[  fc(x3=1, x2=0, x1=0, )fb(x2=0, )fa(x1=0, ) + fc(x3=1, x2=1, x1=0, )fb(x2=1, )fa(x1=0, )  + fc(x3=1, x2=0, x1=1, )fb(x2=0, )fa(x1=1, )  + fc(x3=1, x2=1, x1=1, )fb(x2=1, )fa(x1=1, )][fd(x3=1, x4=0, ) + fd(x3=1, x4=1, )]]",
"mu(fe->x5:1) = [fe(x3=0, x5=1, )[  fc(x3=0, x2=0, x1=0, )fb(x2=0, )fa(x1=0, ) + fc(x3=0, x2=1, x1=0, )fb(x2=1, )fa(x1=0, )  + fc(x3=0, x2=0, x1=1, )fb(x2=0, )fa(x1=1, )  + fc(x3=0, x2=1, x1=1, )fb(x2=1, )fa(x1=1, )][fd(x3=0, x4=0, ) + fd(x3=0, x4=1, )] + fe(x3=1, x5=1, )[  fc(x3=1, x2=0, x1=0, )fb(x2=0, )fa(x1=0, ) + fc(x3=1, x2=1, x1=0, )fb(x2=1, )fa(x1=0, )  + fc(x3=1, x2=0, x1=1, )fb(x2=0, )fa(x1=1, )  + fc(x3=1, x2=1, x1=1, )fb(x2=1, )fa(x1=1, )][fd(x3=1, x4=0, ) + fd(x3=1, x4=1, )]]",
"mu(fb->x2:1) = fb(x2=1, )",
"mu(fb->x2:0) = fb(x2=0, )",
"mu(x3->fd:1) = [  fc(x3=1, x2=0, x1=0, )fb(x2=0, )fa(x1=0, ) + fc(x3=1, x2=1, x1=0, )fb(x2=1, )fa(x1=0, )  + fc(x3=1, x2=0, x1=1, )fb(x2=0, )fa(x1=1, )  + fc(x3=1, x2=1, x1=1, )fb(x2=1, )fa(x1=1, )][fe(x3=1, x5=0, ) + fe(x3=1, x5=1, )]",
"mu(x3->fd:0) = [  fc(x3=0, x2=0, x1=0, )fb(x2=0, )fa(x1=0, ) + fc(x3=0, x2=1, x1=0, )fb(x2=1, )fa(x1=0, )  + fc(x3=0, x2=0, x1=1, )fb(x2=0, )fa(x1=1, )  + fc(x3=0, x2=1, x1=1, )fb(x2=1, )fa(x1=1, )][fe(x3=0, x5=0, ) + fe(x3=0, x5=1, )]",
"mu(x3->fe:0) = [  fc(x3=0, x2=0, x1=0, )fb(x2=0, )fa(x1=0, ) + fc(x3=0, x2=1, x1=0, )fb(x2=1, )fa(x1=0, )  + fc(x3=0, x2=0, x1=1, )fb(x2=0, )fa(x1=1, )  + fc(x3=0, x2=1, x1=1, )fb(x2=1, )fa(x1=1, )][fd(x3=0, x4=0, ) + fd(x3=0, x4=1, )]",
"mu(fc->x3:0) = [  fc(x3=0, x2=0, x1=0, )fb(x2=0, )fa(x1=0, ) + fc(x3=0, x2=1, x1=0, )fb(x2=1, )fa(x1=0, )  + fc(x3=0, x2=0, x1=1, )fb(x2=0, )fa(x1=1, )  + fc(x3=0, x2=1, x1=1, )fb(x2=1, )fa(x1=1, )]",
"mu(x3->fe:1) = [  fc(x3=1, x2=0, x1=0, )fb(x2=0, )fa(x1=0, ) + fc(x3=1, x2=1, x1=0, )fb(x2=1, )fa(x1=0, )  + fc(x3=1, x2=0, x1=1, )fb(x2=0, )fa(x1=1, )  + fc(x3=1, x2=1, x1=1, )fb(x2=1, )fa(x1=1, )][fd(x3=1, x4=0, ) + fd(x3=1, x4=1, )]",
"mu(fc->x3:1) = [  fc(x3=1, x2=0, x1=0, )fb(x2=0, )fa(x1=0, ) + fc(x3=1, x2=1, x1=0, )fb(x2=1, )fa(x1=0, )  + fc(x3=1, x2=0, x1=1, )fb(x2=0, )fa(x1=1, )  + fc(x3=1, x2=1, x1=1, )fb(x2=1, )fa(x1=1, )]",
"mu(x2->fb:0) = [  fc(x3=0, x2=0, x1=0, )[fd(x3=0, x4=0, ) + fd(x3=0, x4=1, )][fe(x3=0, x5=0, ) + fe(x3=0, x5=1, )]fa(x1=0, ) + fc(x3=1, x2=0, x1=0, )[fd(x3=1, x4=0, ) + fd(x3=1, x4=1, )][fe(x3=1, x5=0, ) + fe(x3=1, x5=1, )]fa(x1=0, )  + fc(x3=0, x2=0, x1=1, )[fd(x3=0, x4=0, ) + fd(x3=0, x4=1, )][fe(x3=0, x5=0, ) + fe(x3=0, x5=1, )]fa(x1=1, )  + fc(x3=1, x2=0, x1=1, )[fd(x3=1, x4=0, ) + fd(x3=1, x4=1, )][fe(x3=1, x5=0, ) + fe(x3=1, x5=1, )]fa(x1=1, )]",
"mu(x2->fb:1) = [  fc(x3=0, x2=1, x1=0, )[fd(x3=0, x4=0, ) + fd(x3=0, x4=1, )][fe(x3=0, x5=0, ) + fe(x3=0, x5=1, )]fa(x1=0, ) + fc(x3=1, x2=1, x1=0, )[fd(x3=1, x4=0, ) + fd(x3=1, x4=1, )][fe(x3=1, x5=0, ) + fe(x3=1, x5=1, )]fa(x1=0, )  + fc(x3=0, x2=1, x1=1, )[fd(x3=0, x4=0, ) + fd(x3=0, x4=1, )][fe(x3=0, x5=0, ) + fe(x3=0, x5=1, )]fa(x1=1, )  + fc(x3=1, x2=1, x1=1, )[fd(x3=1, x4=0, ) + fd(x3=1, x4=1, )][fe(x3=1, x5=0, ) + fe(x3=1, x5=1, )]fa(x1=1, )]",
"mu(fe->x3:1) = [fe(x3=1, x5=0, ) + fe(x3=1, x5=1, )]",
"mu(fd->x4:0) = [fd(x3=0, x4=0, )[  fc(x3=0, x2=0, x1=0, )fb(x2=0, )fa(x1=0, ) + fc(x3=0, x2=1, x1=0, )fb(x2=1, )fa(x1=0, )  + fc(x3=0, x2=0, x1=1, )fb(x2=0, )fa(x1=1, )  + fc(x3=0, x2=1, x1=1, )fb(x2=1, )fa(x1=1, )][fe(x3=0, x5=0, ) + fe(x3=0, x5=1, )] + fd(x3=1, x4=0, )[  fc(x3=1, x2=0, x1=0, )fb(x2=0, )fa(x1=0, ) + fc(x3=1, x2=1, x1=0, )fb(x2=1, )fa(x1=0, )  + fc(x3=1, x2=0, x1=1, )fb(x2=0, )fa(x1=1, )  + fc(x3=1, x2=1, x1=1, )fb(x2=1, )fa(x1=1, )][fe(x3=1, x5=0, ) + fe(x3=1, x5=1, )]]",
"mu(fd->x4:1) = [fd(x3=0, x4=1, )[  fc(x3=0, x2=0, x1=0, )fb(x2=0, )fa(x1=0, ) + fc(x3=0, x2=1, x1=0, )fb(x2=1, )fa(x1=0, )  + fc(x3=0, x2=0, x1=1, )fb(x2=0, )fa(x1=1, )  + fc(x3=0, x2=1, x1=1, )fb(x2=1, )fa(x1=1, )][fe(x3=0, x5=0, ) + fe(x3=0, x5=1, )] + fd(x3=1, x4=1, )[  fc(x3=1, x2=0, x1=0, )fb(x2=0, )fa(x1=0, ) + fc(x3=1, x2=1, x1=0, )fb(x2=1, )fa(x1=0, )  + fc(x3=1, x2=0, x1=1, )fb(x2=0, )fa(x1=1, )  + fc(x3=1, x2=1, x1=1, )fb(x2=1, )fa(x1=1, )][fe(x3=1, x5=0, ) + fe(x3=1, x5=1, )]]",
"mu(x2->fc:1) = fb(x2=1, )",
"mu(fe->x3:0) = [fe(x3=0, x5=0, ) + fe(x3=0, x5=1, )]",
"mu(x2->fc:0) = fb(x2=0, )",
"mu(x3->fc:0) = [fd(x3=0, x4=0, ) + fd(x3=0, x4=1, )][fe(x3=0, x5=0, ) + fe(x3=0, x5=1, )]",
};
  return strarray;
}

TEST(test_sumproduct, kshischang_et_al) {
  bool generate_expected_output = false;
  if (generate_expected_output)
    std::cout << " Construct a graph described in Example 1  of "
      << " Factor Graphs and the Sum-Product Algorithm by Frank R Kshischang et al "
      << " (2001)" << std::endl;
  typedef std::pair<int, int> Edge;
  Edge edge_array[] = { Edge(fa, x1), Edge(fc, x3),
    Edge(fb, x2), Edge(fc, x2), Edge(fc, x1), Edge(fd, x3),
    Edge(fd, x4), Edge(fe, x3), Edge(fe, x5) };

  FactorGraph g(edge_array,
      edge_array + sizeof(edge_array) / sizeof(Edge),
       10 );
  //std::cout << "Vertices:" << num_vertices(g) << std::endl;

  typedef typename boost::graph_traits<FactorGraph>::vertex_iterator vertex_iterator;
  vertex_iterator v, v_end;
  boost::tie(v, v_end) = vertices(g);
  std::size_t count = 0;
  boost::unordered_map<Vertex, bool> is_factor_map;
  IsFactorMap is_factor(is_factor_map);
  for (; v != v_end; ++v)
    put(is_factor, *v, (count++ < fa) ? false : true);

  boost::unordered_map<Vertex, FactorType> factors_map;
  FactorMap fmap(factors_map);
  boost::tie(v, v_end) = vertices(g);
  typedef typename  boost::graph_traits<FactorGraph>::vertex_iterator vertex_iterator;
  typedef typename  boost::graph_traits<FactorGraph>::vertex_descriptor vertex_descriptor;
  typedef typename  boost::graph_traits<FactorGraph>::edge_descriptor edge_descriptor;
  for (; v!=v_end;++v) {
    vertex_descriptor vd(*v);
    // Construct a vector of neigbors from out_edges
    // std::vector<vertex_descriptor> nbrs;
    // typedef typename std::vector<vertex_descriptor>::const_iterator nbr_iterator;
    // occgrid::neighbors<FactorGraph>(vd, g, std::back_inserter(nbrs));
    typedef typename boost::graph_traits<FactorGraph>::adjacency_iterator adjacency_iterator;
    std::pair<adjacency_iterator, adjacency_iterator> nbrs( adjacent_vertices(vd, g));

    // std::cout << "Neigbours of " << vd << " are:";
    // for (adjacency_iterator nbr(nbrs.first); nbr != nbrs.second; ++nbr) std::cout << *nbr << "," ;
    // std::cout << std::endl;

    vertex_factor<adjacency_iterator> vf(vd, nbrs.first, nbrs.second);
    put(fmap, *v, vf);
  }

  MessageValuesBaseType msg_values;

  codomain_iter_pair cdip(codomain_static_function());
  BinaryCodomainMap cdmap;
  MessageValues msgs(msg_values);
  typedef typename boost::graph_traits<FactorGraph>::edge_iterator edge_iterator;
  edge_iterator e, e_end;

  for(boost::tie(e, e_end) = edges(g); e != e_end; ++e)
    for (codomain_iterator cd = cdip.first; cd != cdip.second; ++cd)
      put(msgs, typename MessageValues::key_type(source(*e, g), target(*e, g), *cd), 1);

  occgrid::sumproduct_visitor<
    FactorGraph,
    MessageValues,
    BinaryCodomainMap,
    FactorMap,
    IsFactorMap>
      spvis (msgs, cdmap, fmap, is_factor);

  occgrid::single_i_algorithm_traversal(g, spvis);

  const std::string* expected_str = expected_output();
  typedef typename boost::unordered_map<MessageKeyType, SymReal>::const_iterator map_iterator;
  for (map_iterator it(msg_values.cbegin()); it != msg_values.cend(); ++it) {
    Vertex factor = it->first.first.first;
    Vertex var = it->first.first.second;
    sample_space_type cd = it->first.second;
    std::stringstream ss;
    ss << "mu(" << vertex_name(factor) << "->" << vertex_name(var) << ":" << cd << ") = " << it->second;
    if (generate_expected_output)
      std::cout << "\"" << ss.str() << "\"," << std::endl;
    else
      ASSERT_EQ(*expected_str, ss.str());
    expected_str++;
  }
}

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
