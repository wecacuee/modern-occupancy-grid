/**
 *      @file slowMetropolis.cpp
 *      @date May 23, 2012
 *      @author Brian Peasley
 *      @author Frank Dellaert
 */

#include "../../include/OccupancyGrid.h"

/**
 * @brief Run a metropolis sampler.
 * @param iterations defines the number of iterations to run.
 * @return  vector of marginal probabilities.
 */
OccupancyGrid::Marginals runSlowMetropolis(const OccupancyGrid &occupancyGrid, size_t iterations){
	OccupancyGrid::Occupancy occupancy = occupancyGrid.emptyOccupancy();

	size_t size = occupancyGrid.cellCount();
	OccupancyGrid::Marginals marginals(size);

	boost::random::mt19937 rng;
	boost::random::uniform_int_distribution<Index> random_cell(0,size-1);


	// run Metropolis for the requested number of operations
	// compute initial probability of occupancy grid, P(x_t)

	double Px = occupancyGrid(occupancy);

	for(size_t it = 0; it < marginals.size(); it++)
		marginals[it] = 0;

	vector<double> energy;

	for(size_t it = 0; it < iterations; it++){
		energy.push_back(Px);
		printf("%lf\n", (double)it/(double)iterations);
		//choose a random cell
		Index x = random_cell(rng);
		//printf("%ld:",x);
		//flip the state of a random cell, x
			 occupancy[x] = 1 - occupancy[x];

		//compute probability of new occupancy grid, P(x')
		//by summing over all LaserFactor::operator()
			 double Px_prime = occupancyGrid(occupancy);

		//occupancy.print();
		//calculate acceptance ratio, a
			double a = Px_prime/Px;

		//if a <= 1 otherwise accept with probability a
		//if we accept the new state P(x_t) = P(x')
		//	printf(" %.3lf %.3lf\t", Px, Px_prime);
			if(a <= 1){
				Px = Px_prime;
			}
			else{
				 occupancy[x] = 1 - occupancy[x];
			}

		//increment the number of iterations each cell has been on
			for(size_t i = 0; i < size; i++){
				if(occupancy[i] == 1)
					marginals[i]++;
			}
	}

	FILE *fptr = fopen("Data/Metropolis_Energy.txt","w");
		for(int i = 0; i < iterations; i++){
			fprintf(fptr, "%lf ", energy[i]);
		}

	//compute the marginals
	for(size_t it = 0; it < size; it++)
		marginals[it] /= iterations;

	return marginals;
}
