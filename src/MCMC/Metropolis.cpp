/**
 *      @file slowMetropolis.cpp
 *      @date May 23, 2012
 *      @author Brian Peasley
 *      @author Frank Dellaert
 */

#include "OccupancyGrid/OccupancyGrid.h"
#include "OccupancyGrid/visualiser.h"
#include "OccupancyGrid/TwoAssumptionAlgorithm.h"
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

using namespace std;
using namespace gtsam;

/**
 * @brief Run a metropolis sampler.
 * @param iterations defines the number of iterations to run.
 * @return  vector of marginal probabilities.
 */
OccupancyGrid::Marginals runSlowMetropolis(const OccupancyGrid &occupancyGrid,
    size_t iterations, double max_clock) {

  // Create a data structure to hold the estimated marginal occupancy probabilities
  // and initialize to zero.
  size_t size = occupancyGrid.cellCount();
  OccupancyGrid::Marginals marginals(size);
  for (size_t it = 0; it < marginals.size(); it++)
    marginals[it] = 0;

  // Initialize randum number generator
  size_t nrows = occupancyGrid.height();
  //size_t ncols = occupancyGrid.width();
  boost::mt19937 rng;
  boost::uniform_int < Index > random_cell(0, size - 1);
  double sigma = 0.05 * nrows ; // next sample point will be within 2*5 cells (95% of the times)

  boost::normal_distribution< double > normal_dist(0, sigma);
  boost::variate_generator<boost::mt19937&,
    boost::normal_distribution< double > > var_nor(rng, normal_dist);

  // double dsize   = static_cast<double>(size);
  // double dheight = floor(sqrt(dsize));
  // size_t height  = static_cast<size_t>(dheight);
  // size_t width   = static_cast<size_t>(floor(dsize/dheight));
  clock_t st = clock();

  // Create empty occupancy as initial state and
  // compute initial neg log-probability of occupancy grid, - log P(x_t)
  LaserFactor::Occupancy occupancy = occupancyGrid.emptyOccupancy();
  std::vector<double> two_energy(occupancyGrid.cellCount());
  two_assumption_algorithm(occupancyGrid, occupancy, two_energy); 

  double Ex = occupancyGrid(occupancy);
  global_vis_.init(occupancyGrid.height(), occupancyGrid.width());

  // for logging
  vector<double> energy;

  // Choose a random cell
  //Index x_;// = random_cell(rng);

  // run Metropolis for the requested number of operations
  for (size_t it = 0; it < iterations && clock() < (max_clock - st); it++) {

    // Log and print
    energy.push_back(Ex);
    if (it % 100 == 0) {
      clock_t et = clock();
      std::cout << "<Energy>\t" << ((float)(et - st)) / CLOCKS_PER_SEC << "\t" << Ex << std::endl;
      printf("%lf\n", (double) it / (double) iterations);

      if (it % 10000) {
        global_vis_.enable_show();
        global_vis_.reset();
        global_vis_.setMarginals(marginals);
        global_vis_.show();
      }
    }

    // Sample a point close to the previous point with gaussian probability
    // This is a heuristic strategy that high occupancy regions (high
    // probability) are going to be few but close together in space. This is
    // not same as the choosing the proposal distribution for metropolis
    // algorithm as the space we are working is a 100x100 dimensional space
    // rather than a 2D space. But some of the properties of this 2D space can
    // be made use of. This idea is similar to that of a heat map.
    // double col = x % ncols;
    // double row = x / ncols;
    // row += var_nor();
    // col += var_nor();
    // Index row_lu = (row < 0) ? 0
    //   : (row >= nrows) ? nrows - 1
    //   : static_cast<Index>(row);
    // Index col_lu = (col < 0) ? 0
    //   : (col >= ncols) ? ncols - 1
    //   : static_cast<Index>(col);
    // Index x_prime = row_lu * occupancyGrid.width() + col_lu;
    Index x_prime = random_cell(rng);


    // Compute neg log-probability of new occupancy grid, -log P(x')
    // by summing over all LaserFactor::operator()
    double oldValue = occupancy[x_prime];
    double deltaEx = 
      occupancyGrid.computeDelta(occupancy, x_prime, 1 - occupancy[x_prime]);
    assert(occupancy[x_prime] == oldValue);

    // Calculate acceptance ratio, a
    // See e.g. MacKay 96 "Intro to Monte Carlo Methods"
    // a = P(x')/P(x) = exp {-E(x')} / exp {-E(x)} = exp {E(x)-E(x')}
    double a = exp(- deltaEx);

    // If a <= 1 otherwise accept with probability a
    double rn = static_cast<double>(std::rand()) / (RAND_MAX);
    bool accept = (a>=1) ? true // definitely accept
      : (a >= rn) ?  true       // accept with probability a
      : false;

    //printf("%lu : %lu; accepted: %d\n", x_prime, occupancy.at(x_prime), accept);
    if (accept) {
      Ex += deltaEx;
      //x_ = x_prime;
      // we accept: flip it !
      occupancy[x_prime] = 1 - occupancy[x_prime];
    } else {
      //x_ = random_cell(rng);
    }

    //increment the number of iterations each cell has been on
    for (size_t i = 0; i < size; i++) {
      if (occupancy[i] == 1)
        marginals[i]++;
    }
  }

  FILE *fptr = fopen("Data/Metropolis_Energy.txt", "w");
  for (size_t i = 0; i < iterations; i++)
    fprintf(fptr, "%lf ", energy[i]);

  //compute the marginals
  for (size_t it = 0; it < size; it++)
    marginals[it] /= iterations;

  return marginals;
}
