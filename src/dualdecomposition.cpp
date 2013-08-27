#include "OccupancyGrid/OccupancyGrid.h"
#include "OccupancyGrid/loadData.h"
#include "OccupancyGrid/dualdecomposition.hpp"
#include "OccupancyGrid/visualiser.h"
#include "OccupancyGrid/gtsam_adaptor.hpp"
#include "OccupancyGrid/slaveminimizer.hpp"
#include "OccupancyGrid/DDLaserFactor.hpp"

#include <gtsam/geometry/Pose2.h>

Visualiser global_vis_;
Visualiser global_vis2_;
Visualiser global_vis3_;

using namespace std;
using namespace boost;
using namespace gtsam;
using namespace occgrid;

int main(int argc, const char *argv[])
{
  global_vis_.enable_show();
  global_vis2_.enable_show();
  global_vis3_.enable_show();
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
  global_vis2_.init(occupancyGrid.height(), occupancyGrid.width(), "d");
  global_vis3_.init(occupancyGrid.height(), occupancyGrid.width(), "e");
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
  typedef G<OccupancyGrid, double, gtsam::Index, size_t> OccupancyGridGraph;
  typedef _MultiAssignment<OccupancyGridGraph>::base_type MultiAssignmentBaseType;
  typedef _MultiAssignment<OccupancyGridGraph>::property_map_type MultiAssignment;
  typedef typename OccupancyGridGraph::MessageTypes::base_type MessagesBaseType;
  typedef typename OccupancyGridGraph::MessageValues Messages;
  typedef SlaveMinimizer<OccupancyGridGraph,
          DDLaserFactor<MultiAssignment, Messages>, Messages, MultiAssignment> SlaveMinimizer_;
  OccupancyGridGraph ogg(occupancyGrid);
  MultiAssignmentBaseType multiassignbase;
  MultiAssignment multiassign(multiassignbase);
  MessagesBaseType msg_base(0);
  Messages msgs(msg_base);
  SlaveMinimizer_ slvmin(ogg);
  typename OccupancyGridGraph::SampleSpaceMap ssm = get(sample_space_t(), ogg);
  DualDecomposition<OccupancyGridGraph, SlaveMinimizer_,
    typename OccupancyGridGraph::SampleSpaceMap,
    Messages,
    MultiAssignment > dd(msgs, multiassign);
  dd(ogg, slvmin, ssm, 20);
}
