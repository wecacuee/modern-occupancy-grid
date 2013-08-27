/**
 *      @file slowMetropolis.cpp
 *      @date May 23, 2012
 *      @author Brian Peasley
 *      @author Frank Dellaert
 */

#include "OccupancyGrid/OccupancyGrid.h"
#include "OccupancyGrid/TwoAssumptionAlgorithm.h"
#include "OccupancyGrid/visualiser.h"

using namespace std;
using namespace gtsam;

/**
 * @brief Run a metropolis sampler.
 * @param iterations defines the number of iterations to run.
 * @return  vector of marginal probabilities.
 */
OccupancyGrid::Marginals runDDMCMC(const OccupancyGrid &occupancyGrid, size_t iterations){
  clock_t st = clock();
	LaserFactor::Occupancy occupancy = occupancyGrid.emptyOccupancy();
  std::vector<double> two_energy(occupancyGrid.cellCount());
  two_assumption_algorithm(occupancyGrid, occupancy, two_energy); 
	OccupancyGrid::HeatMap heatMap = occupancyGrid.heatMap();

	//create a pdf map to sample from
		vector<Index> map;
    assert(heatMap.size() == occupancyGrid.cellCount());
		for(size_t i = 0; i < heatMap.size(); i++){
			for(size_t j = 0; j < heatMap[i]; j++) {
        assert(i < occupancyGrid.cellCount());
				map.push_back(i);
      }
		}

	//set up random number generator
		size_t size = occupancyGrid.cellCount();
		OccupancyGrid::Marginals marginals(size);

		boost::mt19937 rng;
		boost::uniform_int<Index> random_cell(0, map.size() - 1);


	// run Metropolis for the requested number of operations
	// compute initial probability of occupancy grid, P(x_t)

  double Ex = occupancyGrid(occupancy);
  global_vis_.init(occupancyGrid.height(), occupancyGrid.width());
  global_vis_.enable_show();

	for(size_t it = 0; it < marginals.size(); it++)
		marginals[it] = 0.0;


	vector<double> energy;

	for(size_t it = 0; it < iterations; it++){
		energy.push_back(Ex);
    if (it % 2000 == 0) {
      printf("%lf\n", (double)it/(double)iterations);
      clock_t et = clock();
      std::cout << "<Energy>\t" << ((float)(et - st)) / CLOCKS_PER_SEC << "\t" << Ex << std::endl;
      global_vis_.reset();
      global_vis_.setMarginals(marginals);
      global_vis_.show();
    }
		//choose a random cell
		Index j = random_cell(rng);

		//compute probability of new occupancy grid, P(x')
		//by summing over all LaserFactor::operator()
    Index cellidx = map.at(j);
    assert(cellidx < occupancyGrid.cellCount());
    double deltaEx = 
      occupancyGrid.computeDelta(occupancy, cellidx, 1 - occupancy[cellidx]);

    // Calculate acceptance ratio, a
    // See e.g. MacKay 96 "Intro to Monte Carlo Methods"
    // a = P(x')/P(x) = exp {-E(x')} / exp {-E(x)} = exp {E(x)-E(x')}
    double a = exp(-deltaEx);
      
    // If a <= 1 otherwise accept with probability a
    double rn = static_cast<double>(std::rand()) / (RAND_MAX);
    bool accept = (a>=1) ? true // definitely accept
      : (a >= rn) ?  true       // accept with probability a
      : false;

		//if a <= 1 otherwise accept with probability a
		//if we accept the new state P(x_t) = P(x')
    //	printf(" %.3lf %.3lf\t", Px, Px_prime);
    if(accept) {
      Ex += deltaEx; 
      occupancy[cellidx] = 1 - occupancy[cellidx];
    }

    //increment the number of iterations each cell has been on
    for(size_t i = 0; i < size; i++){
      if(occupancy[i] == 1)
        marginals[i]++;
    }
	}

	FILE *fptr = fopen("Data/DDMCMC_Energy.txt","w");
	for(size_t i = 0; i < iterations; i++){
		fprintf(fptr, "%lf ", energy[i]);
	}

	//compute the marginals
	for(size_t it = 0; it < size; it++)
		marginals[it] /= iterations;

	return marginals;
}
