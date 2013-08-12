/**
 * @brief Defines functions for sumproduct algorithm
 * @file sumproduct.hpp
 * @author Vikas Dhiman
 */
#pragma once
#include "cartesian_product.hpp"
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/lambda/construct.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/range/any_range.hpp>
#include <boost/iterator/iterator_categories.hpp>
#include <boost/random.hpp>
#include <boost/graph/random.hpp>
#include <boost/typeof/typeof.hpp>
#include <boost/foreach.hpp>
#include <sstream>
#include "utility.hpp"
#include "sumproduct_traits.hpp"

namespace occgrid {

/**
 * \brief normalizes values in `map` over InputIterator to sum up to one
 */
// TODO: just use a transform iterator over a map to just accept a Iterator
template<typename InputIterator, typename PropertyMap>
void normalize(InputIterator first, InputIterator last, PropertyMap& map) {
  typedef typename boost::property_traits<PropertyMap>::value_type value_type;
  value_type normalizer(0);
  for (;first != last; ++first) 
    normalizer += map[*first];

  for (;first != last; ++first) 
    map[*first] /= normalizer;
}

namespace detail {
  /**
   * @brief Computes belief message over variable to factor edge.
   *
   * @type FactorGraph: A adjacency list graph.
   * @type MessageValues: A property map denoting messages on graph edges per
   * variable state.
   * @type SampleSpaceMap: Map prodviding sample space of each variable in the
   * graph node.
   * @type FactorMap: Map providing factor function for each factor node.
   *
   * @param e: the edge
   * @param fg: the graph
   * @param msgs: Messages over directed edges for each state of connected
   * variables
   * @param cdmap: Sample space map.
   * @param fmap: Factor map
   * @param xv: The fixed state of source variable node
   * @param v2f_edge_tag: For tag dispatch
   */
  template<typename FactorGraph, typename MessageValues,
    typename SampleSpaceMap,
    typename FactorMap>
  typename MessageValues::value_type
    belief(
      typename boost::graph_traits<FactorGraph>::edge_descriptor e,
      const FactorGraph &fg, MessageValues &msgs,
      SampleSpaceMap &cdmap, FactorMap& fmap,
      const typename SampleSpaceMap::value_type::first_type::value_type &xv,
      v2f_edge_tag) {
    // Short alias
    typedef typename boost::graph_traits<FactorGraph>::vertex_descriptor vertex_descriptor;
    // Deduce real type
    typedef typename MessageValues::value_type Real;

    vertex_descriptor var = source(e, fg), factor = target(e, fg);
    // \prod_{h \in n(var) \ factor} msgs_{h -> var(var = xv)}
    Real prod (1);
    BOOST_FOREACH(vertex_descriptor h, adjacent_vertices(var, fg)) {
      if (h != factor)
        prod *= msgs[typename MessageValues::key_type(h, var, xv)];
    }
    return prod;
  }

  /**
   * @brief Compute a single term inside the summary function.
   *
   * f(X) \prod_{y in n(factor) except var} msgs[y, factor, (y = X_y)]
   *
   * where X is the assignment
   *
   * @type FactorGraph: A adjacency list graph.
   * @type MessageValues: A property map denoting messages on graph edges per
   * variable state.
   * graph node.
   * @type FactorMap: Map providing factor function for each factor node.
   *
   * @param g: the graph
   * @param msgs: Messages over directed edges for each state of connected
   * variables
   * @param fmap: Factor map
   * @param e: the edge
   */
  template<typename FactorGraph,
    typename MessageValues,
    typename FactorMap>
  struct belief_from_var_assignment 
  : public std::unary_function<
    typename FactorMap::value_type::argument_type,
    typename MessageValues::value_type >  
  {
    typedef typename boost::graph_traits<FactorGraph>::vertex_descriptor vertex_descriptor;
    typedef typename FactorMap::value_type::argument_type argument_type;
    typedef typename MessageValues::value_type result_type;
    belief_from_var_assignment(const FactorGraph& g,
        const MessageValues& msgs,
        const FactorMap& fmap,
        const typename boost::graph_traits<FactorGraph>::edge_descriptor e)
      : msgs_(msgs), e_(e), factor_(source(e, g)), var_(target(e, g)),
      func_(get(fmap, factor_)), nbrs_(adjacent_vertices(factor_, g)) {

      }
    result_type operator()(argument_type assign) {
      result_type prod = func_(assign);
      BOOST_FOREACH(vertex_descriptor nbr, nbrs_) {
        if (nbr != var_)
          prod *= msgs_[typename MessageValues::key_type(nbr, factor_, assign[nbr])];
      }
      return prod;
    }
    private:
    const MessageValues& msgs_;
    const typename boost::graph_traits<FactorGraph>::edge_descriptor e_;
    const typename boost::graph_traits<FactorGraph>::vertex_descriptor factor_, var_;
    const typename boost::property_traits<FactorMap>::value_type func_;
    typedef typename boost::graph_traits<FactorGraph>::adjacency_iterator adjacency_iterator;
    std::pair<adjacency_iterator, adjacency_iterator> nbrs_;
  };

  /**
   * @brief Computes belief message over factor to variable edge.
   *
   * @type FactorGraph: A adjacency list graph.
   * @type MessageValues: A property map denoting messages on graph edges per
   * variable state.
   * @type SampleSpaceMap: Map prodviding sample space of each variable in the
   * graph node.
   * @type FactorMap: Map providing factor function for each factor node.
   *
   * @param e: the edge
   * @param fg: the graph
   * @param msgs: Messages over directed edges for each state of connected
   * variables
   * @param cdmap: Sample space map.
   * @param fmap: Factor map
   * @param xv: The fixed state of source variable node
   * @param v2f_edge_tag: For tag dispatch
   */
  template<typename FactorGraph,
    typename MessageValues,
    typename SampleSpaceMap,
    typename FactorMap>
  typename MessageValues::value_type
  belief(
      typename boost::graph_traits<FactorGraph>::edge_descriptor e,
      const FactorGraph &fg,
      MessageValues &msgs,
      SampleSpaceMap &cdmap,
      FactorMap& fmap, 
      //const typename Assignment::value_type &xv
      const typename SampleSpaceMap::value_type::first_type::value_type &xv,
      f2v_edge_tag
      ) {
    typedef typename boost::graph_traits<FactorGraph>::vertex_descriptor vertex_descriptor;
    typedef typename MessageValues::value_type Real;
    typedef typename boost::graph_traits<FactorGraph>::adjacency_iterator adjacency_iterator;
    typedef typename SampleSpaceMap::value_type::first_type sample_space_iterator;
    typedef typename FactorMap::value_type::argument_type const_assignment_ref_type;
    typedef typename boost::remove_reference<const_assignment_ref_type>::type const_assignment_type;
    typedef typename boost::remove_const<const_assignment_type>::type assignment_type;
    namespace bl = boost::lambda;
    using boost::source;
    using boost::target;
    using boost::adjacent_vertices;
    vertex_descriptor factor = source(e, fg), var = target(e, fg);
    BOOST_AUTO_TPL(nbrs, adjacent_vertices(factor, fg));
    // Functor to compute belief from a given assignment of neigboring
    // variables.
    // f(X) \prod_{y in n(factor) except var} msgs[y, factor, (y = X_y)]
    const belief_from_var_assignment<FactorGraph, MessageValues, FactorMap> bfv(fg, msgs,
        fmap, e);
    BOOST_AUTO_TPL(func, get(fmap, factor));
    // Marginalize bfv for var = xv over all nbrs
    Real sum = summaryOf<Real, adjacency_iterator, SampleSpaceMap,
         assignment_type>( bfv, nbrs.first, nbrs.second, cdmap, var, xv);
    return sum;
  }
} // namespace detail

/// Dispatch belief depending on edge type 
template<typename FactorGraph, typename MessageValues,
  typename SampleSpaceMap,
  typename FactorMap,
  typename IsFactorMap>
typename MessageValues::value_type
belief(
    typename boost::graph_traits<FactorGraph>::edge_descriptor e,
    const FactorGraph &fg,
    MessageValues &msgs,
    SampleSpaceMap &cdmap,
    FactorMap& fmap,
    IsFactorMap& is_factor,
    const typename SampleSpaceMap::value_type::first_type::value_type &xv)
{
  typedef typename boost::graph_traits<FactorGraph>::vertex_descriptor vertex_descriptor;
  vertex_descriptor u, v;
  using boost::incident;
  boost::tie(u, v) = incident(e, fg);
  if (get(is_factor, u) && (! get(is_factor, v)))
    return detail::belief<FactorGraph, MessageValues, SampleSpaceMap, FactorMap>(
        e, fg, msgs, cdmap, fmap, xv, f2v_edge_tag());
  else if (( ! get(is_factor, u) ) && get(is_factor, v))
    return detail::belief<FactorGraph, MessageValues, SampleSpaceMap, FactorMap>(
        e, fg, msgs, cdmap, fmap, xv, v2f_edge_tag());
  else {
    std::stringstream ss;
    ss << "Not a factor graph. Following edge is invalid:" << u << " - " << v;
    throw std::logic_error(ss.str());
  }
}

/// for each state in sample space compute belief message over edge e
/// normalize the belief messages
template<typename FactorGraph, typename MessageValues,
  typename SampleSpaceMap,
  typename FactorMap,
  typename IsFactorMap>
void sumproduct(
    typename boost::graph_traits<FactorGraph>::edge_descriptor e,
    const FactorGraph &fg,
    MessageValues &msgs,
    SampleSpaceMap &cdmap,
    FactorMap& fmap,
    IsFactorMap& is_factor)
{
    typedef typename boost::graph_traits<FactorGraph>::vertex_descriptor vertex_descriptor;
    typedef typename MessageValues::value_type Real;
    typedef typename SampleSpaceMap::value_type::first_type sample_space_iterator;

    vertex_descriptor factor = source(e, fg), var = target(e, fg);
    sample_space_iterator cd, cd_end;
    boost::tie(cd, cd_end) = get(cdmap, var);
    for (boost::tie(cd, cd_end) = get(cdmap, var); cd != cd_end; ++cd) {
      Real sum = belief(e, fg, msgs, cdmap, fmap, is_factor, *cd);
      msgs[typename MessageValues::key_type(factor, var, *cd)] = sum;
    }

    // normalize
    boost::tie(cd, cd_end) = get(cdmap, var);

    typedef typename MessageValues::key_type msg_key_type;
    namespace bl = boost::lambda;
    BOOST_AUTO_TPL(ctmkt,
          bl::bind(bl::constructor<msg_key_type>(), factor, var, boost::lambda::_1));
    normalize(make_trans_it(cd, ctmkt),
              make_trans_it(cd_end, ctmkt),
              msgs);
}

/**
 * @brief Visitor class for sumproduct algorithm
 */
template<typename FactorGraph, typename MessageValues,
  typename SampleSpaceMap,
  typename FactorMap,
  typename IsFactorMap>
struct sumproduct_visitor : public boost::base_visitor<sumproduct_visitor<FactorGraph, MessageValues, SampleSpaceMap, FactorMap, IsFactorMap> > {
private:
  MessageValues &msgs_;
  SampleSpaceMap &cdmap_;
  FactorMap& fmap_;
  IsFactorMap& isfactor_;

public:
  typedef boost::on_examine_edge event_filter;
  sumproduct_visitor(MessageValues &msgs, 
    SampleSpaceMap &cdmap, FactorMap& fmap, 
    IsFactorMap& isfactor) 
    : msgs_(msgs), cdmap_(cdmap), fmap_(fmap), isfactor_(isfactor) { }

  void operator()(typename boost::graph_traits<FactorGraph>::edge_descriptor e,
      const FactorGraph& fg) {
    //std::cout << "Called for edge" << source(e, fg) << "-" << target(e, fg) << std::endl;
    sumproduct<FactorGraph, MessageValues, SampleSpaceMap, FactorMap, IsFactorMap>(e, fg, msgs_, cdmap_, fmap_, isfactor_);
  }
};

/// Compute marginal of the given node u for value xv
template <typename FactorGraph, typename MessageValues, typename SampleSpaceType>
typename boost::property_traits<MessageValues>::value_type
marginal(const FactorGraph& fg, 
    const MessageValues& msgs,
    typename boost::graph_traits<FactorGraph>::vertex_descriptor u,
    SampleSpaceType xv
    ) {
  typedef typename boost::graph_traits<FactorGraph>::vertex_descriptor vertex_descriptor;
  typedef typename boost::property_traits<MessageValues>::value_type value_type;
  value_type prod(1);
  BOOST_FOREACH(vertex_descriptor v, adjacent_vertices(u, fg))
    prod *= msgs[typename MessageValues::key_type(u, v, xv)];
  return prod;
}


namespace detail {
  template <typename G, typename RandomNumGen>
  typename boost::graph_traits<G>::edge_descriptor
  random_edge(G& g, RandomNumGen& gen, boost::vertex_list_graph_tag, boost::incidence_graph_tag) {
    typedef typename boost::graph_traits<G>::vertex_descriptor vertex_descriptor;
    typedef typename boost::graph_traits<G>::degree_size_type degree_size_type;
    typedef typename boost::graph_traits<G>::out_edge_iterator out_edge_iterator;

    using boost::random_vertex;
    vertex_descriptor v = random_vertex(g, gen);
    BOOST_AUTO(od, out_degree(v, g));
    // keep trying unless we get a vertex with neigbours
    while (od <= 1) {
      v = random_vertex(g, gen);
      od = out_degree(v, g);
    }

    boost::uniform_int<> distrib(0,  od - 1);
    boost::variate_generator<RandomNumGen&, boost::uniform_int<> > rand_gen(gen, distrib);
    degree_size_type n = rand_gen();

    out_edge_iterator i = out_edges(v, g).first;
    return *(boost::next(i, n));
  }

  template <typename G, typename RandomNumGen>
  typename boost::graph_traits<G>::edge_descriptor
  random_edge(G& g, RandomNumGen& gen, boost::edge_list_graph_tag, boost::edge_list_graph_tag) {
    return boost::random_edge(g, gen);
  }
} // namespace detail

/// Random edge iterator over edge list graph or (vertex list and adjacency list graph)
template <typename G, typename RandomNumGen>
typename boost::graph_traits<G>::edge_descriptor
random_edge(G& g, RandomNumGen& gen) {
  typename boost::graph_traits<G>::traversal_category category;
  return detail::random_edge(g, gen, category, category);
}

/** @brief Random edge traversal with visitors */
template<typename FactorGraph, typename Visitors>
void random_edge_traversal(const FactorGraph& g, Visitors& visitor, std::size_t max_iter) {
  boost::mt19937 gen;
  //std::cout << "n:" << n << std::endl;
  for (std::size_t i = 0; i < max_iter; ++i)
    invoke_visitors(visitor, occgrid::random_edge(g, gen), g, boost::on_examine_edge());
}

/** \brief Total edge traversal over a vertex list and adjacency list graph */
template<typename FactorGraph, typename Visitors>
void total_edge_traversal(const FactorGraph& g, Visitors& visitor, size_t sweeps = 2) {
  boost::mt19937 gen;
  //std::cout << "n:" << n << std::endl;
  typedef typename boost::graph_traits<FactorGraph>::edge_descriptor edge_descriptor;
  typedef typename boost::graph_traits<FactorGraph>::vertex_descriptor vertex_descriptor;
  for (size_t i = 0; i < sweeps; ++i) {
    BOOST_FOREACH(vertex_descriptor v, vertices(g)) {
      BOOST_FOREACH(edge_descriptor e, out_edges(v, g)) {
        invoke_visitors(visitor, e, g, boost::on_examine_edge());
      }
    }
  }
}

/** \brief Single i algoirthm traversal for sum product algorithm over trees */
template<typename FactorGraph, typename Visitors>
void single_i_algorithm_traversal(const FactorGraph& g, Visitors& visitor) {
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
          // visitor(*chosen_edge, g); // outgoing edge
          invoke_visitors(visitor, *chosen_edge, g, boost::on_examine_edge());
      }
    }
    swap(degree_one_or_less_vertices_new, degree_one_or_less_vertices);
  }
}

} // namespace occgrid
