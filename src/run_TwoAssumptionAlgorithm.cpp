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

using namespace std;
using namespace gtsam;
using namespace occgrid;

typedef boost::shared_ptr< LaserFactor > LaserFactorPtr;
Visualiser global_vis_;

int main(int argc, const char *argv[])
{

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
