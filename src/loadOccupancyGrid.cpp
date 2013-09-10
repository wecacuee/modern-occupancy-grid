#include "OccupancyGrid/OccupancyGrid.h"
#include "OccupancyGrid/loadData.h"

#include "gtsam/base/types.h"
#include <boost/program_options.hpp>

using namespace std;
using namespace gtsam;
using namespace occgrid;
namespace po = boost::program_options;

OccupancyGrid loadOccupancyGrid(po::variables_map vm) {
  double resolution = vm["resolution"].as<double>();
  std::string datadirectory = vm["dir"].as<std::string>();
  
  vector<Pose2> allposes;
  vector<double> allranges;
  vector<uint8_t> allreflectance;
  //double max_dist;
  // loadDataFromTxt(
  //     "Data/SICK_Odometry.txt",
  //     "Data/SICK_Snapshot.txt",
  //     allposes, allranges,
  //     max_dist);
  
  loadPlayerSim(
      datadirectory + "/laser_pose_all.bin",
      datadirectory + "/laser_range_all.bin",
      datadirectory + "/scan_angles_all.bin",
      datadirectory + "/laser_reflectance_all.bin",
      allposes, allranges, allreflectance);
  double width, height, origin_x, origin_y;
  shiftPoses(allranges, allposes, width, height, origin_x, origin_y);

  std::cout << "Laser spans (" << width << ", " << height << ")\n";
  if (vm.count("width"))
    width = vm["width"].as<double>();
  if (vm.count("height"))
    height = vm["height"].as<double>();

  // Create the occupancy grid data structure
  OccupancyGrid occupancyGrid(width, height, resolution); //default center to middle
  for (size_t i = 0; i < allranges.size(); i++) {
    const Pose2& pose = allposes[i];
    const double range = allranges[i];
    const uint8_t reflectance = allreflectance[i];
    // this is where factors are added into the factor graph
    occupancyGrid.addLaser(pose, range, reflectance); //add laser to grid
  }
  return occupancyGrid;
}
