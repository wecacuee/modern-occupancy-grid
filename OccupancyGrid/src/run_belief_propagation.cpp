#include "gtsam_adaptor.hpp"
#include "sumproduct.hpp"
#include "../OccupancyGrid/include/cvmat_serialization.h"
#include "../OccupancyGrid/include/visualiser.h"
#include "../OccupancyGrid/include/loadData.h"
#include "../OccupancyGrid/include/OccupancyGrid.h"


using namespace std;
using namespace gtsam;
using namespace occgrid;

Visualiser global_vis_;

int main(int argc, const char *argv[])
{
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

  typedef G<OccupancyGrid, LaserFactor> OccupancyGridGraph;
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
  random_edge_traversal(ogg, spvis, 100);
}
