#include "gtsam_adaptor.hpp"
#include "sumproduct.hpp"
#include "../OccupancyGrid/include/cvmat_serialization.h"
#include "../OccupancyGrid/include/visualiser.h"
#include "../OccupancyGrid/include/loadData.h"
#include "../OccupancyGrid/include/OccupancyGrid.h"

#include <opencv2/opencv.hpp>


using namespace std;
using namespace gtsam;
using namespace occgrid;

typedef G<OccupancyGrid, LaserFactor, logdouble> OccupancyGridGraph;
namespace occgrid {
  namespace detail {
      template <>
      typename MessageTypes<OccupancyGridGraph>::property_map_type::value_type
      belief< OccupancyGridGraph,
        typename MessageTypes<OccupancyGridGraph>::property_map_type,
        typename OccupancyGridGraph::SampleSpaceMap,
        typename OccupancyGridGraph::FactorMap>
        (
         typename boost::graph_traits<OccupancyGridGraph>::edge_descriptor e,
         const OccupancyGridGraph  &fg,
         typename MessageTypes<OccupancyGridGraph>::property_map_type &msgs,
         typename OccupancyGridGraph::SampleSpaceMap &cdmap,
         typename OccupancyGridGraph::FactorMap& fmap, 
         //const typename Assignment::value_type &xv
         const typename OccupancyGridGraph::SampleSpaceMap::value_type::first_type::value_type &xv,
         f2v_edge_tag
        ) {
          BOOST_AUTO_TPL(factor, source(e, fg));
          BOOST_AUTO_TPL(cell, target(e, fg));
          return fg.compute_belief(factor, cell, xv, msgs);
        }
  }
}

Visualiser global_vis_;

template <typename FactorGraph, typename MessageValues>
struct display_peridically_visitor 
: public boost::base_visitor<display_peridically_visitor<FactorGraph, MessageValues> >
{
  typedef boost::on_examine_edge event_filter;
  display_peridically_visitor(const MessageValues& msgs) : msgs_(msgs), count_(0) {}
  void operator()(typename boost::graph_traits<FactorGraph>::edge_descriptor e,
      const FactorGraph& fg) {
    if (count_ % 1000 == 0)
      display(fg);
    ++count_;
  }
  void display(const FactorGraph& fg) {
    typedef typename boost::graph_traits<FactorGraph>::vertex_descriptor vertex_descriptor;
    typedef typename boost::property_traits<MessageValues>::value_type value_type;
    std::vector<value_type> marginals(num_variables(fg));
    size_t count = 0;
    BOOST_FOREACH(vertex_descriptor u, variables(fg)) {
      value_type prod_0(1);
      value_type prod_1(1);
      BOOST_FOREACH(vertex_descriptor v, adjacent_vertices(u, fg)) {
        prod_0 *= msgs_[typename MessageValues::key_type(u, v, 0)];
        prod_1 *= msgs_[typename MessageValues::key_type(u, v, 1)];
      }
      marginals[count ++] = prod_1 / (prod_0 + prod_1);
    }
    fg.display(global_vis_, marginals);
  }
  private:
  const MessageValues& msgs_;
  int count_;
};

int main(int argc, const char *argv[])
{
  global_vis_.enable_show();
  cv::namedWindow("c", CV_WINDOW_NORMAL);
  // parse arguments
  if (argc != 4) {
    printf("ERROR [USAGE]: executable <width (in m)> <height (in m)> <resolution (in m)>");
    exit(1);
  }
  double width = atof(argv[1]); //meters
  double height = atof(argv[2]); //meters
  double resolution = atof(argv[3]); //meters

  // Create the occupancy grid data structure
  OccupancyGrid occupancyGrid(width, height, resolution); //default center to middle
  global_vis_.init(occupancyGrid.height(), occupancyGrid.width());
  vector<Pose2> allposes;
  vector<double> allranges;
  double max_dist;
  loadPlayerSim(
      "Data/player_sim/laser_pose_all.bin",
      "Data/player_sim/laser_range_all.bin",
      "Data/player_sim/scan_angles_all.bin",
      allposes, allranges, max_dist);
  for (size_t i = 0; i < allranges.size(); i++) {
    const Pose2& pose = allposes[i];
    const double range = allranges[i];
    // this is where factors are added into the factor graph
    occupancyGrid.addLaser(pose, range); //add laser to grid
  }

  typename MessageTypes<OccupancyGridGraph>::base_type msg_values;
  typename MessageTypes<OccupancyGridGraph>::property_map_type msgs(msg_values);
  OccupancyGridGraph ogg(occupancyGrid);
  using occgrid::get;
  OccupancyGridGraph::SampleSpaceMap cdmap = get(sample_space_t(), ogg);
  OccupancyGridGraph::FactorMap fmap = get(factor_map_t(), ogg);
  OccupancyGridGraph::IsFactorMap is_factor = get(is_factor_t(), ogg);

  // getting num_edges(ogg)
  std::cout << "Number of edges:" << num_edges(ogg) << std::endl;

  sumproduct_visitor<
    OccupancyGridGraph,
    MessageTypes<OccupancyGridGraph>::property_map_type,
    OccupancyGridGraph::SampleSpaceMap,
    OccupancyGridGraph::FactorMap,
    OccupancyGridGraph::IsFactorMap>
      spvis (msgs, cdmap, fmap, is_factor);

  display_peridically_visitor<OccupancyGridGraph,
    MessageTypes<OccupancyGridGraph>::property_map_type > display_vis(msgs);

  //BOOST_AUTO(vistor_list, std::make_pair(spvis, display_vis));
  random_edge_traversal(ogg, spvis, num_edges(ogg) / 10.);
  display_vis.display(ogg);
  cv::waitKey(-1);
}
