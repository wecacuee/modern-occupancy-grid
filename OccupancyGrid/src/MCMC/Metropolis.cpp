/**
 *      @file slowMetropolis.cpp
 *      @date May 23, 2012
 *      @author Brian Peasley
 *      @author Frank Dellaert
 */

#include "../../include/OccupancyGrid.h"

using namespace std;
using namespace gtsam;

/**
 * @brief Run a metropolis sampler.
 * @param iterations defines the number of iterations to run.
 * @return  vector of marginal probabilities.
 */
OccupancyGrid::Marginals runSlowMetropolis(const OccupancyGrid &occupancyGrid,
    size_t iterations) {

  // Create a data structure to hold the estimated marginal occupancy probabilities
  // and initialize to zero.
  size_t size = occupancyGrid.cellCount();
  OccupancyGrid::Marginals marginals(size);
  for (size_t it = 0; it < marginals.size(); it++)
    marginals[it] = 0;

  // Initialize randum number generator
  boost::mt19937 rng;
  boost::uniform_int < Index > random_cell(0, size - 1);


  // Create empty occupancy as initial state and
  // compute initial neg log-probability of occupancy grid, - log P(x_t)
  LaserFactor::Occupancy occupancy = occupancyGrid.emptyOccupancy();
  double Ex = occupancyGrid(occupancy);

  // for logging
  vector<double> energy;

  // run Metropolis for the requested number of operations
  for (size_t it = 0; it < iterations; it++) {

    // Log and print
    energy.push_back(Ex);
    if (it % 100 == 0)
      printf("%lf\n", (double) it / (double) iterations);

    // Choose a random cell
    Index x = random_cell(rng);

    // Flip the state of a random cell, x
    occupancy[x] = 1 - occupancy[x];

    // Compute neg log-probability of new occupancy grid, -log P(x')
    // by summing over all LaserFactor::operator()
    // TODO: this is just as inefficient as Vikas' choice to loop over ALL rays !!!
    double Ex_prime = occupancyGrid(occupancy);

    // Calculate acceptance ratio, a
    // See e.g. MacKay 96 "Intro to Monte Carlo Methods"
    // a = P(x')/P(x) = exp {-E(x')} / exp {-E(x)} = exp {E(x)-E(x')}
    double a = exp(Ex - Ex_prime);

    // If a <= 1 otherwise accept with probability a
    bool accept = (a>=1) ? true : (false /* TODO: flip a biased coin */ );
    if (accept)
      Ex = Ex_prime;
    else
      // we don't accept: flip it back !
      occupancy[x] = 1 - occupancy[x];

    //increment the number of iterations each cell has been on
    for (size_t i = 0; i < size; i++) {
      if (occupancy[i] == 1)
        marginals[i]++;
    }
  }

  FILE *fptr = fopen("Data/Metropolis_Energy.txt", "w");
  for (int i = 0; i < iterations; i++)
    fprintf(fptr, "%lf ", energy[i]);

  //compute the marginals
  for (size_t it = 0; it < size; it++)
    marginals[it] /= iterations;

  return marginals;
}
