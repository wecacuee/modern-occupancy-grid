/**
 * @brief  Tests a slow version of MCMC for Occupancy grids
 * @author Brian Peasley
 * @date   ?
 */
#include "OccupancyGrid/MCMC.h"
#include "OccupancyGrid/cvmat_serialization.h"
#include "OccupancyGrid/visualiser.h"
#include "OccupancyGrid/loadData.h"
#include <opencv2/opencv.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

using namespace std;
using namespace gtsam;

Visualiser global_vis_;

/// Main
int main(int argc, char *argv[]) {

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
  occupancyGrid.saveLaser("Data/lasers.lsr");

  //run metropolis
  OccupancyGrid::Marginals occupancyMarginals = runSlowMetropolis(occupancyGrid,
      150000);

  // write the result
  char marginalsOutput[1000];
  sprintf(marginalsOutput, "Data/Metropolis_Marginals.txt");
  FILE* fptr = fopen(marginalsOutput, "w");
  fprintf(fptr, "%lu %lu\n", occupancyGrid.width(), occupancyGrid.height());

  for (size_t i = 0; i < occupancyMarginals.size(); i++) {
    fprintf(fptr, "%lf ", occupancyMarginals[i]);
  }
  fclose(fptr);
  global_vis_.save("/tmp/SICKSlowMetropolis.png");
}

