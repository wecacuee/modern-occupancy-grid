#include "OccupancyGrid/OccupancyGrid.h"
#include "OccupancyGrid/loadData.h"
#include "OccupancyGrid/dualdecomposition.hpp"
#include "OccupancyGrid/visualiser.h"
#include "OccupancyGrid/gtsam_adaptor.hpp"
#include "OccupancyGrid/slaveminimizer.hpp"
#include "OccupancyGrid/DDLaserFactor.hpp"
#include "OccupancyGrid/utility.hpp"

#include <gtsam/geometry/Pose2.h>

Visualiser global_vis_;
Visualiser global_vis2_;

using namespace std;
using namespace boost;
using namespace gtsam;
using namespace occgrid;


int main(int argc, const char *argv[])
{
  global_vis_.enable_show();
  global_vis2_.enable_show();
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
  typedef double EnergyType;
  typedef gtsam::Index SampleSpaceType;
  typedef gtsam::Index vertex_descriptor;
  typedef G<OccupancyGrid, EnergyType, SampleSpaceType, vertex_descriptor> OccupancyGridGraph;
  typedef hash_property_map< std::pair<vertex_descriptor, vertex_descriptor>, SampleSpaceType> MultiAssignment;
  typedef DisagreementTracker<OccupancyGridGraph, MultiAssignment> DisagreementMap;
  typedef hash_property_map< typename OccupancyGridGraph::MessageTypes::key_type, EnergyType> Messages;
  typedef SlaveMinimizer<OccupancyGridGraph,
          DDLaserFactor<DisagreementMap, Messages>, Messages, DisagreementMap> SlaveMinimizer_;
  OccupancyGridGraph ogg(occupancyGrid);
  SlaveMinimizer_ slvmin(ogg);
  typedef typename OccupancyGridGraph::SampleSpaceMap SampleSpaceMap;
  SampleSpaceMap ssm = get(sample_space_t(), ogg);
  Messages messages(0);
  MultiAssignment multiassign(0);
  DisagreementMap disagreement_map(ogg, multiassign);
  typedef SubgradientDualDecomposition<OccupancyGridGraph, SlaveMinimizer_,
          SampleSpaceMap, DisagreementMap, Messages, SampleSpaceType, EnergyType> SubgradientDualDecompositionType;
  SubgradientDualDecompositionType subg_dd(ogg, slvmin, ssm, disagreement_map, messages, 50);
  iterate_dualdecomposition<OccupancyGridGraph, SubgradientDualDecompositionType, DisagreementMap, SampleSpaceType>(ogg, subg_dd, disagreement_map, 70);
}
