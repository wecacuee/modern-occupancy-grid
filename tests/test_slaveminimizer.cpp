#include <boost/unordered_map.hpp>
#include "OccupancyGrid/DDLaserFactor.hpp"
typedef size_t vertex_descriptor;
typedef std::pair<size_t, size_t> edge_descriptor;
typedef double energy_type;
typedef size_t sample_space_type;
struct msg_key_type : public std::pair<edge_descriptor, sample_space_type> {
  msg_key_type(vertex_descriptor u, vertex_descriptor v, sample_space_type s) :
    std::pair<edge_descriptor, sample_space_type>(std::make_pair(u, v), s) {}
};
typedef boost::unordered_map<msg_key_type, energy_type> MessageBaseType;
typedef boost::associative_property_map<MessageBaseType> MessageType;

typedef boost::unordered_map<std::pair<vertex_descriptor, vertex_descriptor>, sample_space_type> MultiAssignmentBaseType;
typedef boost::associative_property_map<MultiAssignmentBaseType> MultiAssignment;

int main(int argc, const char *argv[])
{
  MessageBaseType msg_base;
  MessageType msgs(msg_base);

  MultiAssignmentBaseType massign_base;
  MultiAssignment massign(massign_base);

  vertex_descriptor carr[] = { 4888, 4999, 5001, 5002, 5003, 5004 };
  std::vector<vertex_descriptor> cells(carr, carr+6);
  size_t lf_idx = 5000;
  BOOST_FOREACH(vertex_descriptor c, cells) {
    msgs[msg_key_type(lf_idx, c, 0)] = 0;
    msgs[msg_key_type(lf_idx, c, 1)] = 0;
  }
  LaserFactor lf(cells, lf_idx, true);
  DDLaserFactor<MultiAssignment, MessageType> ddlf(lf);
  ddlf(massign, msgs);
  std::cout << "massign:";
  BOOST_FOREACH(vertex_descriptor c, cells) {
    std::cout << c << ":" << massign[std::make_pair(lf_idx, c)] << ",";
  }
  std::cout << std::endl;

  msgs[msg_key_type(lf_idx, 5003, 0)] = 900;
  msgs[msg_key_type(lf_idx, 5004, 1)] = 900;
  ddlf(massign, msgs);
  std::cout << "massign:";
  BOOST_FOREACH(vertex_descriptor c, cells) {
    std::cout << c << ":" << massign[std::make_pair(lf_idx, c)] << ",";
  }
  std::cout << std::endl;

  
  return 0;
}
