#include "OccupancyGrid/OccupancyGrid.h"
#include "OccupancyGrid/loadData.h"
#include "OccupancyGrid/dualdecomposition.hpp"
#include "OccupancyGrid/visualiser.h"
#include "OccupancyGrid/gtsam_adaptor.hpp"
#include "OccupancyGrid/slaveminimizer.hpp"
#include "OccupancyGrid/DDLaserFactor.hpp"
#include "OccupancyGrid/utility.hpp"

#include <gtsam/geometry/Pose2.h>
#include <boost/program_options.hpp>

Visualiser global_vis_;
Visualiser global_vis2_;

using namespace std;
using namespace boost;
using namespace gtsam;
using namespace occgrid;

namespace po = boost::program_options;

int main(int argc, const char *argv[])
{
  typedef double EnergyType;
  global_vis_.enable_show();
  global_vis2_.enable_show();
  // parse arguments ///////////////////////////////////////////
  // Declare the supported options.
  po::options_description desc("Run dual decomposition");
  desc.add_options()
    ("help", "produce help message")
    ("resolution", po::value<double>()->required(), "Size of square cell in the map")
    ("step", po::value<EnergyType>()->default_value(50), "step size for algorithm")
    ("dir", po::value<std::string>()->default_value("Data/player_sim_with_reflectance"), "Data directory")
;

  po::positional_options_description pos;
  pos.add("resolution", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  po::notify(vm);    

  double resolution = vm["resolution"].as<double>();
  EnergyType step = vm["step"].as<EnergyType>();
  std::string datadirectory = vm["dir"].as<std::string>();
  // end of parse arguments ////////////////////////////////////

  vector<Pose2> allposes;
  vector<double> allranges;
  vector<uint8_t> allreflectance;
  loadPlayerSim(
      datadirectory + "/laser_pose_all.bin",
      datadirectory + "/laser_range_all.bin",
      datadirectory + "/scan_angles_all.bin",
      datadirectory + "/laser_reflectance_all.bin",
      allposes, allranges, allreflectance);
  double width, height, origin_x, origin_y;
  shiftPoses(allranges, allposes, width, height, origin_x, origin_y);

  // Create the occupancy grid data structure
  OccupancyGrid occupancyGrid(width, height, resolution); //default center to middle
  global_vis_.init(occupancyGrid.height(), occupancyGrid.width());
  global_vis2_.init(occupancyGrid.height(), occupancyGrid.width(), "d");

  for (size_t i = 0; i < allranges.size(); i++) {
    const Pose2& pose = allposes[i];
    const double range = allranges[i];
    const uint8_t reflectance = allreflectance[i];
    // this is where factors are added into the factor graph
    occupancyGrid.addLaser(pose, range, reflectance); //add laser to grid
  }
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
  SubgradientDualDecompositionType subg_dd(ogg, slvmin, ssm, disagreement_map, messages, step);
  iterate_dualdecomposition<OccupancyGridGraph, SubgradientDualDecompositionType, DisagreementMap, SampleSpaceType>(ogg, subg_dd, disagreement_map, 70);
}
