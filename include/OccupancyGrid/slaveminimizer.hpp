#pragma once
// needed for dual decomposition
template <typename G, typename Factor, typename Messages, typename MultiAssignment>
struct SlaveMinimizer {
  private:
    const G& g_;
  public:
    typedef Factor value_type;
    typedef value_type reference;
    typedef typename boost::graph_traits<G>::vertex_descriptor key_type;
    typedef boost::readable_property_map_tag category;
    SlaveMinimizer(const G& g) : g_(g) { }
    reference get(key_type key) const {
      return Factor(*(g_.g_.factorFromNodeId(key)));
    }
};
