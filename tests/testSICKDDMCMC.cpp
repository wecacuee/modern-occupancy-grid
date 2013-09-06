#include "OccupancyGrid/OccupancyGrid.h"
#include "OccupancyGrid/MCMC.h"
#include "OccupancyGrid/visualiser.h"
#include "OccupancyGrid/loadData.h"
#include <boost/program_options.hpp>
namespace po = boost::program_options;

using namespace std;
using namespace gtsam;
Visualiser global_vis_;

int main(int argc, char *argv[]){
  // parse arguments ///////////////////////////////////////////
  // Declare the supported options.
  po::options_description desc("Run dual decomposition");
  desc.add_options()
    ("help", "produce help message")
    ("resolution", po::value<double>()->required(), "Size of square cell in the map")
    ("dir", po::value<std::string>()->default_value("Data/player_sim_with_reflectance"), "Data directory")
;

  po::positional_options_description pos;
  pos.add("resolution", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  po::notify(vm);    

  double resolution = vm["resolution"].as<double>();
  std::string datadirectory = vm["dir"].as<std::string>();
  // end of parse arguments ////////////////////////////////////
  vector<Pose2> allposes;
  vector<double> allranges;
  vector<uint8_t> allreflectance;
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
  double width, height;
  shiftPoses(allranges, allposes, width, height);

	OccupancyGrid occupancyGrid(width, height, resolution); //default center to middle

  global_vis_.init(occupancyGrid.height(), occupancyGrid.width());
  for (size_t i = 0; i < allranges.size(); i++) {
    const Pose2& pose = allposes[i];
    const double range = allranges[i];
    const uint8_t reflectance = allreflectance[i];
    // this is where factors are added into the factor graph
    occupancyGrid.addLaser(pose, range, reflectance); //add laser to grid
  }

  occupancyGrid.saveLaser("Data/lasers.lsr");
  occupancyGrid.saveHeatMap("Data/HeatMap.ppm");

	//run metropolis
	OccupancyGrid::Marginals occupancyMarginals = runDDMCMC(occupancyGrid, 70000);

	char marginalsOutput[1000];
			sprintf(marginalsOutput, "Data/DDMCMC_Marginals.txt");
	FILE* fptr = fopen(marginalsOutput, "w");
	fprintf(fptr, "%lu %lu\n", occupancyGrid.width(), occupancyGrid.height());

	for(size_t i = 0; i < occupancyMarginals.size(); i++){
		fprintf(fptr, "%lf ", occupancyMarginals[i]);
	}
	fclose(fptr);
  global_vis_.save("/tmp/SICKDDMCMC.png");
}


