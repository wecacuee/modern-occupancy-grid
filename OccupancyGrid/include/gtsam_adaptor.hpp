/**
 * \brief Adapt gtsam::DiscreteFactorGraph and it's descendants to a boost
 * graph that is model of following concepts:
 *
 * IncidenceGraph
 * AdjancencyGraph
 * VertexListGraph
 * EdgeListGraph
 * AdjacencyMatrix
 * PropertyGraph
 */

// Talk::
// What is gtsam::DiscreteFactorGraph
// factors_ : a vector of factor nodes
//
// it has been specialized by OccupancyGrid which adds the following
// information
//
// cell2factors_ : edges from cells to factors_ . Since the edges are
// undirected, these are the only edges we have.
//
// the vertex descriptors are boost::shared_ptrs of type
// gtsam::DiscreteFactors . These classes can also contain some of the
// required properties of factor type
//
// We don't really need a variable(cell)  class, but we have decide for a
// single type of vertex_descriptor. Probably we can create another level of
// redirection for factors
//
// So here goes the design:
//
// vertex_descriptor is gtsam::Index
//
// cells is gtsam::Index, but we need to know the cellCount()
//
// factors gtsam::Index, factors start from cellCount() + 1
//
// IsFactorMap:
// is_factor is simply a function mapping w.r.t. cellCount()
// 
// FactorMap
// factor_pointers_ : gtsam::Index -> shared_ptr to gtsam::DiscreteFactors
// cell2factors_ needs to be changed to gtsam::Index -> gtsam::Index
//
// SampleSpaceMap in this case is a constant function
//
// edge_descriptor is just std::pair<vertex_descriptor, vertex_descriptor>
//
// MessageValues is also sorted out

#include <gtsam/discrete/DiscreteFactorGraph.h> 
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/functional.hpp>
#include <boost/bind.hpp>
#include <boost/unordered_map.hpp>
#include <boost/iterator/counting_iterator.hpp>

namespace occgrid {
  struct gtsam_traversal_category : 
    public virtual boost::adjacency_graph_tag, 
    public virtual boost::incidence_graph_tag,
    public virtual boost::vertex_list_graph_tag {
    };

  struct sample_space_t {
    typedef boost::vertex_property_tag kind;
  };
  struct is_factor_t { 
    typedef boost::vertex_property_tag kind;
  };
  struct factor_map_t { 
    typedef boost::vertex_property_tag kind;
  };
  template <typename gtsamGraph, typename Factor>
  class G {
    private:
      const gtsamGraph& g_;
    public:
    typedef typename Factor::argument_type::mapped_type sample_space_type;
    typedef gtsam::Index vertex_descriptor;
    typedef std::pair<gtsam::Index, gtsam::Index> edge_descriptor;
    typedef boost::undirected_tag directed_category;
    typedef gtsam_traversal_category traversal_category;
    typedef size_t vertices_size_type;
    typedef size_t degree_size_type;
    typedef typename std::vector<vertex_descriptor>::const_iterator adjacency_iterator;

    template<typename T1, typename T2>
    class bound_make_pair {
      private:
        T1 t1_;
      public:
        typedef typename std::pair<T1, T2> result_type;
        typedef T2 argument_type;
        bound_make_pair(T1 t1) : t1_(t1) { }
       result_type operator()(argument_type t2) { 
        return std::make_pair(t1_, t2); 
      }
    };
    typedef boost::function< std::pair<vertex_descriptor, vertex_descriptor> (vertex_descriptor) > Function;
    typedef typename boost::transform_iterator<Function, adjacency_iterator> out_edge_iterator;
    typedef boost::counting_iterator<vertex_descriptor> vertex_iterator;
    // I don't know why we need them?
    typedef size_t edges_size_type;
    typedef typename std::vector<edge_descriptor>::iterator in_edge_iterator;
    typedef typename std::vector<edge_descriptor>::iterator edge_iterator;
    typedef typename boost::disallow_parallel_edge_tag edge_parallel_category;

    G(const gtsamGraph& g) : g_(g) {};

    std::pair<adjacency_iterator, adjacency_iterator> 
    adjacent_vertices(vertex_descriptor v) const
    {
      return std::make_pair(g_.getNeighbors(v).begin(), g_.getNeighbors(v).end());
    }


    std::pair<out_edge_iterator, out_edge_iterator> 
      out_edges(vertex_descriptor v) const
      { 
        bound_make_pair<vertex_descriptor, vertex_descriptor> pair_make(v);
        Function f = pair_make;
        return std::make_pair(
            out_edge_iterator(g_.getNeighbors(v).begin(), f),
            out_edge_iterator(g_.getNeighbors(v).end(), f));
      }


    degree_size_type
      out_degree(vertex_descriptor v)  const
      {
        return g_.getNeighbors(v).size();
      }

    vertices_size_type
      num_vertices() const {
        return g_.cellCount() + g_.factorCount();
      }

    bool is_factor(vertex_descriptor v) const {
      return g_.is_factor(v);
    }

    inline boost::function<double (typename Factor::argument_type)>
      factorFromNodeId(gtsam::Index node_idx) const {
        return *(g_.factorFromNodeId(node_idx));
    }

    struct SampleSpaceMap {
      typedef typename Factor::argument_type::mapped_type ValueType;
      typedef std::vector<ValueType> vector_type;
      typedef typename std::vector<ValueType>::const_iterator const_iterator;
      typedef std::pair<const_iterator, const_iterator> value_type;
      typedef std::pair<const_iterator, const_iterator> reference;
      typedef vertex_descriptor key_type;
      typedef boost::readable_property_map_tag category;

      reference
      get(key_type key) const {
        typedef typename Factor::argument_type::mapped_type ValueType;
        const static ValueType binary[] = {0, 1};
        const static vector_type binary_vector(binary, binary + 2);
        return std::make_pair(binary_vector.begin(), binary_vector.end());
      }
    };

    class IsFactorMap {
      private:
        const G& g_;
      public:
      typedef bool value_type;
      typedef bool reference;
      typedef vertex_descriptor key_type;
      typedef boost::readable_property_map_tag category;
      IsFactorMap(const G& g) : g_(g) { }
      reference get(key_type key) const {
        return g_.is_factor(key) ;
      }
    };

    struct FactorMap {
      private:
        const G& g_;
      public:
      typedef boost::function<double (typename Factor::argument_type)> value_type;
      typedef const value_type reference;
      typedef vertex_descriptor key_type;
      typedef boost::readable_property_map_tag category;
      FactorMap(const G& g) : g_(g) { }
      reference get(key_type key) const {
        return g_.factorFromNodeId(key);
      }
    };
  };
  template <typename G>
    struct MessageTypes {
      typedef typename G::sample_space_type sample_space_type;
      typedef typename G::vertex_descriptor vertex_descriptor;
      struct key_type :
        public std::pair< std::pair<vertex_descriptor, vertex_descriptor>, sample_space_type> {
        explicit key_type(vertex_descriptor u, vertex_descriptor v, sample_space_type ss) : 
          std::pair< std::pair<vertex_descriptor, vertex_descriptor>, sample_space_type>(
              std::pair<vertex_descriptor, vertex_descriptor>(u, v), ss) {};
      };
      typedef boost::unordered_map<key_type, double> base_type;
      typedef boost::associative_property_map<base_type>
        property_map_type;
    };

  template <typename G>
  std::pair<typename G::adjacency_iterator, typename G::adjacency_iterator> 
  adjacent_vertices(typename G::vertex_descriptor v, const G& g) 
  {
    return g.adjacent_vertices(v);
  }

  template <typename G>
  std::pair<typename G::out_edge_iterator, typename G::out_edge_iterator> 
  out_edges(typename G::vertex_descriptor v, const G& g) 
  {
    return g.out_edges(v);
  }

  template <typename G>
  typename G::degree_size_type
  out_degree(typename G::vertex_descriptor v, const G& g) 
  {
    return g.out_degree(v);
  }

  template <typename G>
    typename G::vertices_size_type
    num_vertices(const G& g) {
      return g.num_vertices();
    }

  template <typename G>
    std::pair<typename G::vertex_iterator, typename G::vertex_iterator>
    vertices(const G& g) {
      typedef typename G::vertices_size_type vertices_size_type;
      vertices_size_type n(num_vertices(g));
      return std::make_pair(boost::counting_iterator<vertices_size_type>(0),
                            boost::counting_iterator<vertices_size_type>(n));
    }

  template <typename G>
  typename G::SampleSpaceMap get(sample_space_t, const G& g) {
    typename G::SampleSpaceMap ssm;
    return ssm;
  }

  template <typename G>
  typename G::IsFactorMap get(is_factor_t, const G& g) {
    return typename G::IsFactorMap(g);
  }
  template <typename G>
  typename G::FactorMap get(factor_map_t, const G& g) {
    return typename G::FactorMap(g);
  }

  template <typename PropertyMap>
    typename PropertyMap::reference get(PropertyMap& pm, typename PropertyMap::key_type v) {
      return pm.get(v);
    }

} // namespace occgrid
