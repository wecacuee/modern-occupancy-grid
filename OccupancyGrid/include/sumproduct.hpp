#include "cartesian_product.hpp"
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/range/any_range.hpp>
#include <boost/iterator/iterator_categories.hpp>
#include <sstream>

namespace occgrid {
struct v2f_edge_tag {};
struct f2v_edge_tag {};

template<typename FactorGraph, typename MessageValues,
  typename CodomainMap,
  typename FactorMap>
void sumproduct(
    typename boost::graph_traits<FactorGraph>::edge_descriptor e,
    const FactorGraph &fg, MessageValues &msgs,
    CodomainMap &cdmap, FactorMap& fmap, v2f_edge_tag) {
  typedef typename boost::graph_traits<FactorGraph>::edge_descriptor Edge;
  typedef typename boost::graph_traits<FactorGraph>::vertex_descriptor Vertex;
  typedef typename MessageValues::value_type Real;
  typename boost::graph_traits<FactorGraph>::out_edge_iterator e_it, e_end;
  typename CodomainMap::value_type::first_type cd, cd_end;
  Vertex var = source(e, fg), factor = target(e, fg);

  for (boost::tie(cd, cd_end) = get(cdmap, var); cd != cd_end; ++cd) {
    boost::tie(e_it, e_end) = out_edges(var, fg);
    Real prod = 1;
    for (; e_it != e_end; ++e_it) {
      Vertex opp = opposite(*e_it, var, fg);
      if (opp != factor)
        prod *= get(msgs, std::make_pair(std::make_pair(opp, var), *cd));
    }
    // std::cout << "mu(x" << var + 1 << "->f" << factor << ":" << *cd << ") = " << prod << std::endl;
    put(msgs, std::make_pair(std::make_pair(var, factor), *cd), prod);
  }
}

template<typename G, typename OutputIterator>
void
neighbors(
    typename boost::graph_traits<G>::vertex_descriptor source,
    G g,
    OutputIterator result) 
{
  typedef typename boost::graph_traits<G>::vertex_descriptor Vertex;
  typedef typename boost::graph_traits<G>::vertex_iterator VertexIterator;
  typedef typename boost::graph_traits<G>::out_edge_iterator out_edge_iterator; 
  typedef typename boost::graph_traits<G>::edge_descriptor Edge;
  typedef boost::function<Vertex (Edge)> FunctionType;
  typedef boost::transform_iterator<FunctionType, out_edge_iterator> neighbor_iterator;

  out_edge_iterator e, e_end;
  boost::tie(e, e_end) = out_edges(source, g);
  for (; e != e_end; ++e)
    *result++ = opposite(*e, source, g);
}

template<typename FactorGraph, typename MessageValues,
  typename CodomainMap,
  typename FactorMap>
void sumproduct(
    typename boost::graph_traits<FactorGraph>::edge_descriptor e,
    const FactorGraph &fg,
    MessageValues &msgs,
    CodomainMap &cdmap,
    FactorMap& fmap, 
    f2v_edge_tag) {
  typedef typename boost::graph_traits<FactorGraph>::vertex_descriptor Vertex;

  typedef typename boost::graph_traits<FactorGraph>::out_edge_iterator out_edge_iterator; 
  typedef typename  boost::graph_traits<FactorGraph>::vertex_iterator vertex_iterator;
  typedef typename  boost::graph_traits<FactorGraph>::vertex_descriptor vertex_descriptor;
  typedef typename MessageValues::value_type Real;

  Vertex factor = source(e, fg), var = target(e, fg);

  typedef typename std::iterator_traits<typename CodomainMap::value_type::first_type>::value_type codomain_type;

  typename CodomainMap::value_type::first_type cd;
  typename CodomainMap::value_type::second_type cd_end;
  for (boost::tie(cd, cd_end) = get(cdmap, var); cd != cd_end; ++cd) {
    // Construct a vector of neigbors from out_edges
    // std::vector<vertex_descriptor> nbrs;
    // typedef typename std::vector<vertex_descriptor>::const_iterator nbr_iter;
    // neighbors(var, fg, std::back_inserter(nbrs));
    typedef typename boost::graph_traits<FactorGraph>::adjacency_iterator adjacency_iterator;
    std::pair<adjacency_iterator, adjacency_iterator> nbrs( adjacent_vertices(factor, fg));

    // Construct all possible assignments
    CartesianProduct<adjacency_iterator, CodomainMap> poss_assign(nbrs.first, nbrs.second, cdmap);
    typename FactorMap::value_type::argument_type assign;
    Real sum = 0;
    while (poss_assign.next(assign)) {
      //for (adjacency_iterator nbr(nbrs.first); nbr != nbrs.second; ++nbr) std::cout << *nbr << ":" << assign[*nbr] << "; " << std::endl;
      if (assign[var] != *cd)
        continue;
      Real prod = fmap[factor](assign);
      for (adjacency_iterator nbr(nbrs.first); nbr != nbrs.second; ++nbr) {
        if (*nbr != var)
          prod *= get(msgs, std::make_pair(std::make_pair(*nbr, factor), assign[*nbr]));
      }
      sum += prod;
    }
    // std::cout << "mu(f" << factor << "->x" << var + 1 << ":" << *cd << ") = " << sum << std::endl;
    put(msgs, std::make_pair(std::make_pair(factor, var), *cd), sum);
  }
}

template<typename FactorGraph, typename MessageValues,
  typename CodomainMap,
  typename FactorMap,
  typename IsFactorMap>
void sumproduct(
    typename boost::graph_traits<FactorGraph>::edge_descriptor e,
    const FactorGraph &fg,
    MessageValues &msgs, CodomainMap &cdmap, FactorMap& fmap,
    IsFactorMap& is_factor) {
  typedef typename boost::graph_traits<FactorGraph>::vertex_descriptor Vertex;

  Vertex u, v;
  boost::tie(u, v) = incident(e, fg);
  if (get(is_factor, u) && (! get(is_factor, v)))
    sumproduct<FactorGraph, MessageValues, CodomainMap, FactorMap>(
        e, fg, msgs, cdmap, fmap, f2v_edge_tag());
  else if (( ! get(is_factor, u) ) && get(is_factor, v))
   sumproduct<FactorGraph, MessageValues, CodomainMap, FactorMap>(
       e, fg, msgs, cdmap, fmap, v2f_edge_tag());
  else {
    std::stringstream ss;
    ss << "Not a factor graph:" << u << " - " << v;
    throw std::logic_error(ss.str());
  }
}

template<typename FactorGraph, typename MessageValues,
  typename CodomainMap,
  typename FactorMap,
  typename IsFactorMap>
struct sumproduct_visitor : public boost::base_visitor<sumproduct_visitor<FactorGraph, MessageValues, CodomainMap, FactorMap, IsFactorMap> > {
private:
  MessageValues &msgs_;
  CodomainMap &cdmap_;
  FactorMap& fmap_;
  IsFactorMap& isfactor_;

public:
  typedef boost::on_examine_edge event_filter;
  sumproduct_visitor(MessageValues &msgs, 
    CodomainMap &cdmap, FactorMap& fmap, 
    IsFactorMap& isfactor) 
    : msgs_(msgs), cdmap_(cdmap), fmap_(fmap), isfactor_(isfactor) { }

  void operator()(typename boost::graph_traits<FactorGraph>::edge_descriptor e,
      const FactorGraph& fg) {
    // std::cout << "Called for edge" << source(e, fg) << "-" << target(e, fg) << std::endl;
    sumproduct<FactorGraph, MessageValues, CodomainMap, FactorMap, IsFactorMap>(e, fg, msgs_, cdmap_, fmap_, isfactor_);
  }
};

/** \brief Single i algoirthm traversal for sum product algorithm over trees */
template<typename FactorGraph, typename Visitor>
void single_i_algorithm_traversal(FactorGraph g, Visitor visitor) {
  typedef typename boost::graph_traits<FactorGraph>::adjacency_iterator adjacency_iterator;
  typedef typename boost::graph_traits<FactorGraph>::vertex_iterator vertex_iterator;
  typedef typename boost::graph_traits<FactorGraph>::vertex_descriptor vertex_descriptor;
  typedef typename boost::graph_traits<FactorGraph>::degree_size_type degree_size_type;
  typedef typename boost::graph_traits<FactorGraph>::out_edge_iterator out_edge_iterator;
  typedef typename boost::graph_traits<FactorGraph>::edge_descriptor edge_descriptor;
  typedef typename std::pair<vertex_descriptor, vertex_descriptor> vertex_pair;
  typedef typename boost::unordered_map<vertex_pair, bool> MessageSentType;
  typedef typename MessageSentType::const_iterator vpair_iterator;
  vertex_iterator v, v_end;

  // The is the number of incoming edges from a vertex has NOT received any
  // message so far
  boost::unordered_map<vertex_descriptor, degree_size_type> message_degree;

  // On each "directed"* edge, the message will be sent only once. This map
  // keeps a track whether the message has been sent or not 
  // *(that's why we use a pair of vertices instead of edge descriptor)
  MessageSentType message_sent;

  // Keeps a collection of vertices for next iteration 
  std::vector<vertex_descriptor> degree_one_or_less_vertices;

  // Initialize degrees for each vertex
  for (boost::tie(v, v_end) = vertices(g); v != v_end; ++v) {
    degree_size_type d = out_degree(*v, g);
    message_degree[*v] = d;
    if (d == 1)
      degree_one_or_less_vertices.push_back(*v);
  }

  typedef typename std::vector<vertex_descriptor>::const_iterator d_one_iterator;
  //int count = 1;
  while (degree_one_or_less_vertices.size() > 0) {
    std::vector<vertex_descriptor> degree_one_or_less_vertices_new;
    //std::cout << "Step " << count++ << "+++++++++++++++++++++++++" << std::endl;
    for (d_one_iterator it(degree_one_or_less_vertices.begin());
        it != degree_one_or_less_vertices.end();
        ++it) 
    {
      // When the message_degree of a vertex is 1, it means that we have
      // messages available from all sides but one. 
      // - In that case we find the "incoming edge" from where this vertex
      //   did NOT receive a message  and send the message in that direction. 
      //   There should be only one such edge.
      // - Once the message_degree of a vertex is 0, it means that the
      //   vertex has received messages from all sides and is free to send
      //   to message to all "outgoing edges" that have NOT been sent messages
      //   so far.
      out_edge_iterator e, e_end;
      std::vector<edge_descriptor> filtered_edges;
      for (boost::tie(e, e_end) = out_edges(*it, g); e != e_end; ++e) {
        vertex_descriptor t = target(*e, g);

        assert(message_degree[*it] == 0 || message_degree[*it] == 1);
        vpair_iterator vpair = (message_degree[*it] == 1) 
          ? message_sent.find(std::make_pair(t, *it))  // incoming edge
          : message_sent.find(std::make_pair(*it, t)); // outgoing edge

        if (vpair == message_sent.end())
          filtered_edges.push_back(*e);
      }

      // Send messages along these edges and update the data structures
      for (typename std::vector<edge_descriptor>::const_iterator 
          chosen_edge(filtered_edges.begin());
          chosen_edge != filtered_edges.end(); ++chosen_edge)
      {
          vertex_descriptor t = target(*chosen_edge, g);

          // if message available this edge is already processed
          // std::cout << "examining edge:" << *it << " -> " << t << std::endl;
          message_sent[incident(*chosen_edge, g)] = true; // outgoing edge

          // prepair for next iteration
          message_degree[t] -= 1;
          if (message_degree[t] == 0 || message_degree[t] == 1) 
            degree_one_or_less_vertices_new.push_back(t);

          // Actually send message
          visitor(*chosen_edge, g); // outgoing edge
      }
    }
    swap(degree_one_or_less_vertices_new, degree_one_or_less_vertices);
  }
}


} // namespace occgrid
