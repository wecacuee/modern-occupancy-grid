/**
 * @file : test_cartesian_product.hpp
 * @author : Vikas Dhiman
 * @date: Wed July 11 2013
 */

#include "OccupancyGrid/cartesian_product.hpp"

#include <boost/bind.hpp>
#include <cstdio>
#include <vector>
#include <iostream>
#include <gtest/gtest.h>

#include <boost/property_map/property_map.hpp>
#include <boost/unordered_map.hpp>
#include <boost/type_traits.hpp>

using namespace occgrid;

typedef size_t nodeId_t;
typedef size_t Value;
typedef boost::unordered_map<size_t, Value> Assignment;
/****************************************************************
 * Testing
 ****************************************************************/
template<int Radix>
class RadixNode {
  typedef typename std::vector< Value >::const_iterator PossibleValueIter;
  typedef typename std::pair<PossibleValueIter, PossibleValueIter> iter_pair;
  private:
    const static std::vector<Value> poss_values_;
    nodeId_t node_id_;
    nodeId_t next_node_id() {
      static nodeId_t inc_node_id(0);
      nodeId_t tmpnode = inc_node_id;
      inc_node_id ++;
      return tmpnode;
    }
  public:
    RadixNode() : node_id_(next_node_id()) {
    }

    RadixNode(nodeId_t nid) : node_id_(nid) {}
    const iter_pair possible_values() const {
      return std::make_pair(poss_values_.begin(), poss_values_.end());
    }
    nodeId_t nodeId() const { return node_id_; }
    virtual ~RadixNode() {};
};


Value pv10[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
template<>
const std::vector<Value> RadixNode<10>::poss_values_(pv10, pv10 + sizeof(pv10) / sizeof(Value));

Value pv2[] = {0, 1};
template<>
const std::vector<Value> RadixNode<2>::poss_values_(pv2, pv2 + sizeof(pv2) / sizeof(Value));

typedef RadixNode<2> BinaryNode;
typedef RadixNode<10> DecimalNode;

typedef std::size_t codomain_type;
typedef typename std::vector<codomain_type>::const_iterator codomain_iterator;
typedef std::pair<codomain_iterator, codomain_iterator> codomain_iter_pair;
codomain_iter_pair codomain_static_function() {
  const static std::size_t codomain_array[] = {0, 1, 2, 3, 4, 5, 6, 7,8 ,9};
  const static std::vector<codomain_type> codomain_vector(
      codomain_array, codomain_array + 10);
  return std::make_pair(codomain_vector.begin(), codomain_vector.end());
}
/// Codomain Property Map for variable nodes
class DecimalCodomainMap {
  public:
  typedef nodeId_t key_type;
  typedef codomain_iter_pair value_type;
  typedef value_type& reference;
  typedef boost::lvalue_property_map_tag category;
};
DecimalCodomainMap::value_type get(DecimalCodomainMap c, nodeId_t n) {
  return codomain_static_function();
}

Assignment::mapped_type& get(Assignment& map, const typename Assignment::key_type& k) {
  return map[k];
}
void put(Assignment& map, const typename Assignment::key_type& k, typename Assignment::mapped_type& value) {
  map[k] = value;
}

TEST(CartesianProduct, test1) {
// int main(int argc, const char *argv[])
// {
  std::vector<nodeId_t> bivars;
  for (int i = 0; i < 5; i ++)
    bivars.push_back(i);

  codomain_iter_pair cd( codomain_static_function());
  DecimalCodomainMap cdmap;
  CartesianProduct< std::vector<nodeId_t>::const_iterator, DecimalCodomainMap > 
    poss_assign(bivars.begin(), bivars.end(), cdmap);

  Assignment assign;
  int NDIGIT = 5;
  int num = 0;
  char num_as_str[NDIGIT];
  while (poss_assign.next(assign)) {
    typedef typename std::vector<nodeId_t>::const_iterator iter_t;
    sprintf(num_as_str, "%05d", num++);
    for (iter_t it(bivars.begin()); it != bivars.end(); it++) {
      nodeId_t nid = *it;
      char digit = num_as_str[NDIGIT - nid - 1];
      size_t fulldigit = (size_t) (digit - '0');
      (void) fulldigit;
      ASSERT_EQ(fulldigit, assign[nid]) << "Error at " << nid << ":" << fulldigit << "<>" << assign[nid] << "num:" << std::string(num_as_str);
    }
  }
}

/// Codomain Property Map for variable nodes
class BinaryCodomainMap {
  public:
  typedef nodeId_t key_type;
  typedef codomain_iter_pair value_type;
  typedef value_type& reference;
  typedef boost::lvalue_property_map_tag category;
};
BinaryCodomainMap::value_type get(BinaryCodomainMap c, nodeId_t n) {
  const static std::size_t codomain_array[] = {0, 1};
  const static std::vector<codomain_type> codomain_vector(
      codomain_array, codomain_array + 2);
  return std::make_pair(codomain_vector.begin(), codomain_vector.end());
}

template<typename InputIterator>
struct averageOf 
: public std::unary_function<const Assignment&, double> {
  averageOf(InputIterator begin, InputIterator end) : begin_(begin), end_(end) {}
  double operator() (const Assignment& assign) const {
    double sum = 0;
    for (InputIterator it(begin_); it != end_; ++it)
      sum += const_cast<Assignment&>(assign)[*it];
    double avg =  sum / 5;
    return avg;
  }
  private:
  InputIterator begin_, end_;
};

TEST(summaryOf, test1) {
// int main(int argc, const char *argv[])
// {
  std::vector<nodeId_t> dependent_nodes;
  for  (int i=0;i<5;i++) {
    dependent_nodes.push_back(i);
  }
  //typedef typename UnaryFunction::argument_type PropertyMap;
  //typedef boost::remove_reference<typename UnaryFunction::argument_type>::type PropertyMap;
  typedef std::vector<nodeId_t>::const_iterator InputIterator;
  //typename PropertyMap::mapped_type one(1);
  Value one(1);
  averageOf<InputIterator> func(dependent_nodes.begin(), dependent_nodes.end());
    //boost::bind(averageOf<std::vector<nodeId_t>::const_iterator>, dependent_nodes.begin(), dependent_nodes.end(), _1);
  BinaryCodomainMap cdmap;
  InputIterator dnb = dependent_nodes.begin(), dne = dependent_nodes.end();
  typename InputIterator::value_type x(dependent_nodes[2]);
  double res = summaryOf<double,
         InputIterator, BinaryCodomainMap, Assignment>(func , dnb, dne, cdmap, x, one);
  (void)res;
  ASSERT_EQ(9.6, res);
}

int main(int argc, char *argv[])
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
