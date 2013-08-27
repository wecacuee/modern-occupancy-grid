#pragma once
namespace occgrid {
  struct gtsam_traversal_category : 
    public virtual boost::adjacency_graph_tag, 
    public virtual boost::incidence_graph_tag,
    public virtual boost::vertex_list_graph_tag {
    };

  // internal property names
  struct sample_space_t {
    typedef boost::vertex_property_tag kind;
  };
  struct is_factor_t { 
    typedef boost::vertex_property_tag kind;
  };
  struct factor_map_t { 
    typedef boost::vertex_property_tag kind;
  };
  template <typename gtsamGraph, typename Factor, typename belief_type>
  class G {
    private:
    const gtsamGraph& g_;
    public:
  };
} // namespace occgrid
