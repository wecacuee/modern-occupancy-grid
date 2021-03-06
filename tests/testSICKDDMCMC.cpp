#include "OccupancyGrid/OccupancyGrid.h"
#include "OccupancyGrid/MCMC.h"
#include "OccupancyGrid/visualiser.h"
#include "OccupancyGrid/loadOccupancyGrid.h"
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
    ("width", po::value<double>(), "Width")
    ("height", po::value<double>(), "Height")
    ("resolution", po::value<double>()->required(), "Size of square cell in the map")
    ("dir", po::value<std::string>()->default_value("Data/player_sim_with_reflectance"), "Data directory")
    ("clock", po::value<double>()->default_value(400), "Max clock")
;

  po::positional_options_description pos;
  pos.add("resolution", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  po::notify(vm);    

  std::string directory = vm["dir"].as<std::string>();
  double max_clock = CLOCKS_PER_SEC * vm["clock"].as<double>();
  // end of parse arguments ////////////////////////////////////
  OccupancyGrid occupancyGrid = loadOccupancyGrid(vm);
  global_vis_.init(occupancyGrid.height(), occupancyGrid.width());
  occupancyGrid.saveHeatMap("Data/HeatMap.ppm");

	//run metropolis
	OccupancyGrid::Marginals occupancyMarginals = runDDMCMC(occupancyGrid, 700000, max_clock);

	char marginalsOutput[1000];
			sprintf(marginalsOutput, "Data/DDMCMC_Marginals.txt");
	FILE* fptr = fopen(marginalsOutput, "w");
	fprintf(fptr, "%lu %lu\n", occupancyGrid.width(), occupancyGrid.height());

	for(size_t i = 0; i < occupancyMarginals.size(); i++){
		fprintf(fptr, "%lf ", occupancyMarginals[i]);
	}
	fclose(fptr);
  std::stringstream ss;
  ss << directory << "/SICKDDMCMC.png";
  global_vis_.save(ss.str());
}


