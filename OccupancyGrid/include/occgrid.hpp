#ifndef OCCGRID_HPP
#define OCCGRID_HPP
// c
#include <cmath>
#include <cassert>

// std
#include <algorithm>
#include <iostream>
#include <stdexcept>

// 3rd party
#include <boost/math/constants/constants.hpp>
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include <occgrid.h>

#undef DEBUG

namespace bconst = boost::math::constants;

template <typename real_type, typename int_type>
const uint8_t OccupancyGrid2D<real_type, int_type>::OCCUPIED = 0;

template <>
const uint8_t OccupancyGrid2D<double, int>::OCCUPIED = 0;

template <typename real_type, typename int_type>
const uint8_t OccupancyGrid2D<real_type, int_type>::FREE = 255;

template <>
const uint8_t OccupancyGrid2D<double, int>::FREE = 255;

template <typename real_t, typename int_t>
real_t OccupancyGrid2D<real_t, int_t>::nearest_neighbor_distance(
    cv::Vec<real_t, 2> position,
    real_t max_range,
    cv::Vec<int_t, 2>& nearest_neighbor) 
{
  int_t i = static_cast<int_t>(floor(position(0) - min_pt_(0) / cell_size_(0)));
  int_t j = static_cast<int_t>(floor(position(1) - min_pt_(1) / cell_size_(1)));
  int_t max_range_x = static_cast<int_t>(floor(max_range / cell_size_(0)));
  std::vector<cv::Vec<int_t, 2> > manhattan_neighbours;
  for (int_t r = 0; r < max_range_x; r ++) {
      for (int_t xt = max(i - r, 0); xt <= min(i + r, og_.size[0]-1); xt++) {

	  int_t ry = 
	    static_cast<int_t>(
		floor(((r - fabs(xt - i))* cell_size_(0)) / cell_size_(1)));

	  // y has only two possible values
	  for (int_t yt = j - ry; yt <= j + ry; yt += ((2*ry <= 0) ? 1 : 2*ry)) {
	      //if (DEBUG)
        // printf("(%d, %d), r: %d\n", xt, yt, r);
	      if (yt >= og_.size[1] || yt < 0)
          continue;

	      if (is_occupied(xt, yt)) {
            manhattan_neighbours.push_back(
                cv::Vec<int_t, 2>(xt, yt));
            //if (DEBUG)
            //  printf("^^^^^^^^^^^^^^^^^^^\n");
        }
	  }
      }
      if (manhattan_neighbours.size() > 0) {
	  break;
      }
  }
  real_t min_distance = std::numeric_limits<double>::infinity();
  for (typename std::vector<cv::Vec<int_t, 2> >::iterator it = 
      manhattan_neighbours.begin() ;
      it != manhattan_neighbours.end(); ++it) 
    {
      cv::Vec<real_t, 2> cell_mid_pt = *it;
      cell_mid_pt += cv::Vec<real_t, 2>(0.5, 0.5);
      cell_mid_pt = cell_mid_pt.mul(cell_size_);
      cell_mid_pt += min_pt_;
      real_t dist = cv::norm(cell_mid_pt - position);
      if (min_distance > dist) {
          min_distance = dist;
          nearest_neighbor = *it;
      }
#ifdef DEBUG
        printf("Neigbors: (%d, %d), %f\n", (*it)(0), (*it)(1), dist);
#endif
    }
  return min_distance;
}

template <typename real_t, typename int_t>
real_t OccupancyGrid2D<real_t, int_t>::ray_trace(
    real_t px, 
    real_t py,
    real_t ptheta,
    real_t max_range,
    cv::Vec<real_t, 2>& final_pos) 
{
  // shift coordinates so that we are always in first quadrant
  px = px - min_pt_(0);
  py = py - min_pt_(1);

  real_t dx = cos(ptheta);
  real_t dy = sin(ptheta);
  int_t dirx = signum(dx);
  dirx = (dirx == 0) ? 1 : dirx;

  int_t diry = signum(dy);
  diry = (diry == 0) ? 1 : diry;

  int_t i = static_cast<int_t>(floor(px / cell_size_(0)));

  int_t j = static_cast<int_t>(floor(py / cell_size_(1)));

  // distance to nearest grid line
  int_t floor_or_ceilx = (dirx > 0) ? 1 : 0;
  int_t floor_or_ceily = (diry > 0) ? 1 : 0;
#ifdef DEBUG
      printf("cell: (%i, %i), dxdy:(%f, %f)\n", i, j, dx, dy);
      //std::cout << "cell size:" << cell_size_ << "pos:" << position << std::endl;
#endif
  real_t ex = dirx * ((i + floor_or_ceilx) * cell_size_(0) - px);
  real_t ey = diry * ((j + floor_or_ceily) * cell_size_(1) - py);

  // time to collision from one grid line to another
  real_t Tx = cell_size_(0) / fabs(dx);
  real_t Ty = cell_size_(1) / fabs(dy);

  // time to collision
  real_t tx = ex / fabs(dx);
  real_t ty = ey / fabs(dy);
  // assert(tx >= 0); // time is always positive (for me, for now)
  if ( ! ((tx >= 0) && (ty >= 0))) {

      printf("direction:(%f, %f), position:(%f, %f), cell:(%d, %d), cellsize:(%f, %f)\n", 
          dx, dy, px, py, i, j, cell_size_(0), cell_size_(1));
      throw std::logic_error("tx < 0 or ty < 0");
  }
  assert(tx >= 0);
  assert(ty >= 0);

  // real_t min_cell_size = 
  //   (cell_size_(0) < cell_size_(1)) ?  cell_size_(0) : cell_size_(1);

  real_t dirmag = sqrt(dx*dx + dy*dy); 
  real_t n = floor(max_range * fabs(dx) / dirmag / cell_size_(0)) 
    + floor(max_range * fabs(dy) / dirmag / cell_size_(1));
  int maxsizex = og_.size[0];
  int maxsizey = og_.size[1];

  while (n > 0) {
      n --;

#ifdef DEBUG
        printf("(%d, %d), (%f, %f)\n", i, j, tx, ty);
#endif

      if (tx < ty) {
          i += dirx;
          ty = ty - tx;
          tx = Tx;
      } else {
          j += diry;
          tx = tx - ty;
          ty = Ty;
      }


      if (i < 0 ||  j < 0 || i >= maxsizex || j >= maxsizey ||
          //(og_.at<uint8_t>(i, j) != FREE)) 
          is_occupied(i, j))
        {
          real_t txdx = (dx == 0) ? ex : tx * dx;
          real_t posx = (i +  floor_or_ceilx) * cell_size_(0) - txdx;
          final_pos(0) = posx + min_pt_(0);

          real_t tydy = (dy == 0) ? ey : ty * dy;
          real_t posy = (j +  floor_or_ceily) * cell_size_(1) - tydy;
          final_pos(1) = posy + min_pt_(1);

          real_t disp_x = posx - px;
          real_t disp_y = posy - py;
          return sqrt(disp_x * disp_x + disp_y * disp_y);
      }
  }
  return max_range;
}

///////////////////////////////////////////////////
// Test cases: Uses automatic registration of tests from gtest library
//
// The following macro runs all test in a linked test.cpp main
// return RUN_ALL_TESTS();
///////////////////////////////////////////////////

class OccupancyGrid2DTest : public ::testing::Test {
  protected:
    OccupancyGrid2D<double, int> og_;
    cv::Vec<double, 2> position_;
    double angle_;
    OccupancyGrid2DTest():
      og_(-2.5, -1.5, 0.5, 0.5, 12, 6),
      position_(0, 0),
      angle_(atan2(0.5, 2.5)) {
      }


    virtual void SetUp() {
	og_.og_ = cv::Scalar(og_.FREE);
	og_.og_.at<uint8_t>(10, 4) = og_.OCCUPIED;
	og_.og_.at<uint8_t>(10, 1) = og_.OCCUPIED;
	og_.og_.at<uint8_t>(11, 2) = og_.OCCUPIED;
	og_.og_.at<uint8_t>(11, 3) = og_.OCCUPIED;
	og_.og_.at<uint8_t>(1, 2) = og_.OCCUPIED;
	og_.og_.at<uint8_t>(1, 3) = og_.OCCUPIED;
#ifdef DEBUG
	std::cout << og_.og_ << std::endl;
#endif
    }
};

TEST_F(OccupancyGrid2DTest, testFirstQuadrant) {
    cv::Vec<double, 2> final_pos;
    double range = og_.ray_trace(position_(0), position_(1), angle_, 100,
        final_pos);
    ASSERT_DOUBLE_EQ(sqrt(2.5*2.5 + 0.5*0.5), range);
    ASSERT_DOUBLE_EQ(2.5, final_pos(0));
    ASSERT_DOUBLE_EQ(0.5, final_pos(1));
}

TEST_F(OccupancyGrid2DTest, testFourthQuadrant) {
    cv::Vec<double, 2> final_pos;
    double range = og_.ray_trace(position_(0), position_(1), 
        - angle_, 100, final_pos);
    ASSERT_DOUBLE_EQ(range, sqrt(2.5*2.5 + 0.5*0.5));
    ASSERT_DOUBLE_EQ(final_pos(0), 2.5);
    ASSERT_DOUBLE_EQ(final_pos(1), -0.5);
}


TEST_F(OccupancyGrid2DTest, testThirdQuadrant) {
    cv::Vec<double, 2> final_pos;
    double range = og_.ray_trace(position_(0), position_(1),
        angle_ - bconst::pi<double>(),
	100,
	final_pos);
    double epsilon = 1e-10;
    ASSERT_NEAR(final_pos(0), -1.5, epsilon);
    ASSERT_NEAR(final_pos(1), -0.3, epsilon);
    ASSERT_NEAR(range, sqrt(1.5*1.5 + 0.3*0.3), epsilon);
}

TEST_F(OccupancyGrid2DTest, testSecondQuadrant) {
    cv::Vec<double, 2> final_pos;
    double range = og_.ray_trace(position_(0), position_(1),
        bconst::pi<double>() - angle_,
	100,
	final_pos);
    double eps = 1e-10;
    ASSERT_NEAR(final_pos(0), -1.5, eps);
    ASSERT_NEAR(final_pos(1), 0.3, eps);
    ASSERT_NEAR(range, sqrt(1.5*1.5 + 0.3*0.3), eps);
}

TEST_F(OccupancyGrid2DTest, testHorizontal) {
    cv::Vec<double, 2> final_pos;
    double range = og_.ray_trace(position_(0), position_(1) ,
        0,
	100,
	final_pos);
    ASSERT_DOUBLE_EQ(final_pos(0), 3);
    ASSERT_DOUBLE_EQ(final_pos(1), 0);
    ASSERT_DOUBLE_EQ(range, 3);
}

TEST_F(OccupancyGrid2DTest, testNearestNeighbor) {
    cv::Vec<int, 2> neighbor;
    double dist = og_.nearest_neighbor_distance(
	cv::Vec<double, 2>(1, 0),
	6,
	neighbor);
    ASSERT_DOUBLE_EQ(dist, sqrt(1.75*1.75 + 0.75*0.75));
    ASSERT_EQ(neighbor(0), 10);
    ASSERT_EQ(neighbor(1), 4);
}
#endif // OCCGRID_HPP
