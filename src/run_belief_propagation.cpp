#include "OccupancyGrid/gtsam_adaptor.hpp"
#include "OccupancyGrid/sumproduct.hpp"
#include "OccupancyGrid/cvmat_serialization.h"
#include "OccupancyGrid/visualiser.h"
#include "OccupancyGrid/loadData.h"
#include "OccupancyGrid/OccupancyGrid.h"
#include "OccupancyGrid/BPLaserFactor.hpp"

#include <opencv2/opencv.hpp>


using namespace std;
using namespace gtsam;
using namespace occgrid;

typedef G<OccupancyGrid, logdouble, size_t, gtsam::Index> OccupancyGridGraph;
namespace occgrid {
  namespace detail {
      // full template specialization for overriding the sumproduct update
      // mechanism
      template <> logdouble
      belief
      < OccupancyGridGraph,
        typename OccupancyGridGraph::MessageValues,
        typename OccupancyGridGraph::SampleSpaceMap,
        typename OccupancyGridGraph::FactorMap>
        (
         typename boost::graph_traits<OccupancyGridGraph>::edge_descriptor e,
         const OccupancyGridGraph  &fg,
         typename OccupancyGridGraph::MessageValues &msgs,
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
  display_peridically_visitor(const MessageValues& msgs) : msgs_(msgs), count_(0), clock_start_(clock()) {}
  void operator()(typename boost::graph_traits<FactorGraph>::edge_descriptor e,
      const FactorGraph& fg) {
    if ((count_ % 500000 == 0)) {
      std::cout << "Iterations:" << count_ << std::endl;
      display(fg);
    }
    ++count_;
  }
  void display(const FactorGraph& fg) {
    /// Very very slow
    typedef typename boost::graph_traits<FactorGraph>::vertex_descriptor vertex_descriptor;
    typedef typename boost::property_traits<MessageValues>::value_type value_type;
    std::vector<value_type> margs(num_variables(fg));
    gtsam::Assignment<gtsam::Index> best_assign;
    size_t count = 0;
    BOOST_FOREACH(vertex_descriptor u, variables(fg)) {
      value_type prod_0(marginal(fg, msgs_, u, 0));
      value_type prod_1(marginal(fg, msgs_, u, 1));
      value_type denom = prod_0 + prod_1;
      if (denom == value_type(0)) 
        margs[count] = value_type(0);
      else
        margs[count] = prod_1 / denom;
      best_assign[count] = (margs[count] < value_type(0.5)) ? 0 : 1;
      ++count;
    }
    fg.display(global_vis_, margs);
    double energy = fg.g_(best_assign);
    clock_t et = clock();
    std::cout << "<Energy>\t" << ((float)(et - clock_start_)) / CLOCKS_PER_SEC << "\t" << energy << std::endl;
  }
  private:
  const MessageValues& msgs_;
  int count_;
  clock_t clock_start_;
};

int main(int argc, const char *argv[])
{
  global_vis_.enable_show();
  cv::namedWindow("c", cv::WINDOW_NORMAL);
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
  vector<uint8_t> allreflectance;
  loadPlayerSim(
      "Data/player_sim_with_reflectance/laser_pose_all.bin",
      "Data/player_sim_with_reflectance/laser_range_all.bin",
      "Data/player_sim_with_reflectance/scan_angles_all.bin",
      "Data/player_sim_with_reflectance/laser_reflectance_all.bin",
      allposes, allranges, allreflectance);
  for (size_t i = 0; i < allranges.size(); i++) {
    const Pose2& pose = allposes[i];
    const double range = allranges[i];
    const uint8_t reflectance = allreflectance[i];
    // this is where factors are added into the factor graph
    occupancyGrid.addLaser(pose, range, reflectance); //add laser to grid
  }

  typename OccupancyGridGraph::MessageTypes::base_type msg_values(logdouble(0.5));
  typename OccupancyGridGraph::MessageValues msgs(msg_values);
  OccupancyGridGraph ogg(occupancyGrid);
  using occgrid::get;
  OccupancyGridGraph::SampleSpaceMap cdmap = get(sample_space_t(), ogg);
  OccupancyGridGraph::FactorMap fmap = get(factor_map_t(), ogg);
  OccupancyGridGraph::IsFactorMap is_factor = get(is_factor_t(), ogg);

  // getting num_edges(ogg)

  sumproduct_visitor<
    OccupancyGridGraph,
    OccupancyGridGraph::MessageValues,
    OccupancyGridGraph::SampleSpaceMap,
    OccupancyGridGraph::FactorMap,
    OccupancyGridGraph::IsFactorMap>
      spvis (msgs, cdmap, fmap, is_factor);

  display_peridically_visitor<OccupancyGridGraph,
    typename OccupancyGridGraph::MessageValues > display_vis(msgs);

  BOOST_AUTO(vistor_list, std::make_pair(spvis, display_vis));
  BOOST_AUTO(n_iter, num_edges(ogg) * 0.3);
  std::cout << "Number of iterations:" << n_iter << std::endl;
  random_edge_traversal(ogg, vistor_list, n_iter);
  display_vis.display(ogg);
}
