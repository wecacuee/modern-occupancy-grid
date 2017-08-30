#include "OccupancyGrid/TwoAssumptionAlgorithm.h"
#include "OccupancyGrid/OccupancyGrid.h"
#include "OccupancyGrid/LaserFactor.h"
#include "OccupancyGrid/loadData.h"
#include "OccupancyGrid/loadOccupancyGrid.h"

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
  // double max_clock = CLOCKS_PER_SEC * vm["clock"].as<double>();
  // end of parse arguments ////////////////////////////////////
  OccupancyGrid occupancyGrid = loadOccupancyGrid(vm);
  global_vis_.init(occupancyGrid.height(), occupancyGrid.width());

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
  std::stringstream ss;
  ss << directory << "/TwoAssumptionAlgo.png";
  global_vis_.save(ss.str());
  global_vis_.show(2000);

  return 0;
}
