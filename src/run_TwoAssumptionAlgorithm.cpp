#include "OccupancyGrid/TwoAssumptionAlgorithm.h"
#include "OccupancyGrid/OccupancyGrid.h"
#include "OccupancyGrid/LaserFactor.h"
#include "OccupancyGrid/loadData.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include "OccupancyGrid/visualiser.h"

#include "gtsam/discrete/Assignment.h"
#include "gtsam/base/types.h"
#include <boost/program_options.hpp>

using namespace std;
using namespace gtsam;
using namespace occgrid;
namespace po = boost::program_options;

typedef boost::shared_ptr< LaserFactor > LaserFactorPtr;
Visualiser global_vis_;

int main(int argc, const char *argv[])
{

  cv::namedWindow("c", cv::WINDOW_NORMAL);
  // parse arguments ///////////////////////////////////////////
  // Declare the supported options.
  po::options_description desc("Run dual decomposition");
  desc.add_options()
    ("help", "produce help message")
    ("width", po::value<double>()->required(), "Width of map")
    ("height", po::value<double>()->required(), "Height of map")
    ("resolution", po::value<double>()->required(), "Size of square cell in the map")
    ("dir", po::value<std::string>()->default_value("Data/player_sim_with_reflectance"), "Data directory")
;

  po::positional_options_description pos;
  pos.add("width", 1)
    .add("height", 1)
    .add("resolution", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  po::notify(vm);    

  double width = vm["width"].as<double>();
  double height = vm["height"].as<double>();
  double resolution = vm["resolution"].as<double>();
  std::string datadirectory = vm["dir"].as<std::string>();
  // end of parse arguments ////////////////////////////////////
  
  // Create the occupancy grid data structure
  OccupancyGrid occupancyGrid(width, height, resolution); //default center to middle
  global_vis_.init(occupancyGrid.height(), occupancyGrid.width());
  global_vis_.enable_show();
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

  for (size_t i = 0; i < allranges.size(); i++) {
    const Pose2& pose = allposes[i];
    const double range = allranges[i];
    const uint8_t reflectance = allreflectance[i];
    // this is where factors are added into the factor graph
    occupancyGrid.addLaser(pose, range, reflectance); //add laser to grid
  }
  occupancyGrid.saveLaser("Data/lasers.lsr");
  
  gtsam::Assignment<gtsam::Index> best_assign;
  std::vector<double> energy(occupancyGrid.cellCount());
  clock_t st = clock();
  two_assumption_algorithm(occupancyGrid, best_assign, energy);

  double Ex = occupancyGrid(best_assign);
  clock_t et = clock();
  std::cout << "<Energy>\t" << ((float)(et - st)) / CLOCKS_PER_SEC << "\t" << Ex << std::endl;
  std::vector<double> probab(energy.size());
  for (size_t i = 0; i < energy.size(); ++i) {
    double e = energy[i];
    double p = std::exp(-e);
    probab[i] = 1 / (1 + p);
  }
  global_vis_.setMarginals(probab);
  global_vis_.save("/tmp/TwoAssumptionAlgo.png");
  global_vis_.show(2000);

  return 0;
}
