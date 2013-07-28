/**
 * @brief  Tests a slow version of MCMC for Occupancy grids
 * @author Brian Peasley
 * @date   ?
 */
#include "../OccupancyGrid/include/OccupancyGridCache.h"
#include "../OccupancyGrid/include/MCMC.h"
#include "../OccupancyGrid/include/cvmat_serialization.h"
#include "../OccupancyGrid/include/visualiser.h"
#include "../OccupancyGrid/include/loadData.h"

using namespace std;
using namespace gtsam;

Visualiser global_vis_;

/// Main
int main(int argc, char *argv[]) {

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
  OccupancyGridCache occupancyGrid(width, height, resolution); //default center to middle
  global_vis_.init(occupancyGrid.height(), occupancyGrid.width());
  vector<Pose2> allposes;
  vector<double> allranges;
  double max_dist;
  // loadDataFromTxt(
  //     "Data/SICK_Odometry.txt",
  //     "Data/SICK_Snapshot.txt",
  //     allposes, allranges,
  //     max_dist);
  loadPlayerSim(
      "Data/player_sim/laser_pose_all.bin",
      "Data/player_sim/laser_range_all.bin",
      "Data/player_sim/scan_angles_all.bin",
      allposes, allranges, max_dist);

  for (int i = 0; i < allranges.size(); i++) {
    const Pose2& pose = allposes[i];
    const double range = allranges[i];
    // this is where factors are added into the factor graph
    occupancyGrid.addLaser(pose, range); //add laser to grid
  }
  occupancyGrid.saveLaser("Data/lasers.lsr");

  //run metropolis
  OccupancyGrid::Marginals occupancyMarginals = runSlowMetropolis(occupancyGrid,
      100000);

  // write the result
  char marginalsOutput[1000];
  sprintf(marginalsOutput, "Data/Metropolis_Marginals.txt");
  FILE* fptr = fopen(marginalsOutput, "w");
  fprintf(fptr, "%lu %lu\n", occupancyGrid.width(), occupancyGrid.height());

  for (int i = 0; i < occupancyMarginals.size(); i++) {
    fprintf(fptr, "%lf ", occupancyMarginals[i]);
  }
  fclose(fptr);
}

