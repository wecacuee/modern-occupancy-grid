#include <boost/typeof/typeof.hpp>
#include <iostream>
#include <vector>
#include "OccupancyGrid/raytrace.hpp"
#include <gtest/gtest.h>
using namespace occgrid;

int discretize(double x, double origin, double cellsize) {
  return std::floor((x - origin) / cellsize);
}

void slow_raytrace(
    double px,
    double py,
    double dx, 
    double dy,
    double origin_x,
    double origin_y,
    double cell_size_x,
    double cell_size_y,
    size_t N,
    std::vector<int>& is,
    std::vector<int>& js) 
{
  double step = std::min(cell_size_x, cell_size_y) / 2.;
  double steps_taken = 0;
  int i = 0, j=0;
  for (size_t k = 0; k < N; ++k) {
    i = discretize(px + steps_taken * dx, origin_x, cell_size_x);
    j = discretize(py + steps_taken * dy, origin_y, cell_size_y);
    steps_taken += step;
    while ((k > 0) && ((i == is[k - 1]) && (j == js[k - 1]))) {
      i = discretize(px + steps_taken * dx, origin_x, cell_size_x);
      j = discretize(py + steps_taken * dy, origin_y, cell_size_y);
      steps_taken += step;
    }
    is[k] = i;
    js[k] = j;
  }
}

void test_with_values(double px, double py, double dx, double dy) {
  double cell_size_x = 1, cell_size_y = 1;
  double origin_x = -10, origin_y = 10;
  // std::cout << "pos:(" << px << ", " << py << "),"
  //   << "dir:(" << dx << ", " << dy << ")"
  //   << "cellsize:(" << cell_size_x << ", " << cell_size_y << ")"
  //   << "origin:(" << origin_x << ", " << origin_y << ")" << std::endl;

  size_t N = 10;
  std::vector<int> is(N);
  std::vector<int> js(N);
  slow_raytrace(px, py, dx, dy, origin_x, origin_y, cell_size_x, cell_size_y, N, is, js);

  typedef ray_trace_iterator<double, int> raytrace_it;
  BOOST_AUTO(rt_it, raytrace_it(px, py, dx, dy, origin_x, origin_y, cell_size_x, cell_size_y));
  for (size_t k = 0; k < 10; ++k, ++rt_it) {
    std::stringstream ss;
    ss << "(" << rt_it->first << ", " << rt_it->second << ")"
      << "<=> (" << is[k] << "," << js[k] << ")" << std::endl;
    bool x_test = ((is[k] - 1) <= rt_it->first) && (rt_it->first <= (is[k] + 1));
    ASSERT_TRUE(x_test) ;
    bool y_test = ((js[k] - 1) <= rt_it->second) && (rt_it->second <= (js[k] + 1));
    ASSERT_TRUE(y_test);
  }
}

TEST(test_raytrace, test_random)
{
  double px = (20. * rand() / RAND_MAX - 10);
  double py = (20. * rand() / RAND_MAX - 10);
  double dx = (2. * rand() / RAND_MAX - 1);
  double dy = (2. * rand() / RAND_MAX - 1);
  if ((dx == 0) && (dy == 0))
    dx = 1;
  test_with_values(px, py, dx, dy);
}

TEST(test_raytrace, test_zero_direction)
{
  double px = (20. * rand() / RAND_MAX - 10);
  double py = (20. * rand() / RAND_MAX - 10);
  double dx = (2. * rand() / RAND_MAX - 1);
  double dy = 0;
  if ((dx == 0) && (dy == 0))
    dx = 1;
  test_with_values(px, py, dx, dy);
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
