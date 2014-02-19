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

#include "OccupancyGrid/occgrid.h"
#include "OccupancyGrid/raytrace.hpp"

#undef DEBUG

namespace bconst = boost::math::constants;

template <typename real_type, typename int_type>
const uint8_t OccupancyGrid2D<real_type, int_type>::OCCUPIED = 0;

template <typename real_type, typename int_type>
const uint8_t OccupancyGrid2D<real_type, int_type>::FREE = 255;

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
    cv::Vec<real_t, 2>& final_pos,
    bool& reflectance) 
{
  real_t dx = cos(ptheta);
  real_t dy = sin(ptheta);

  occgrid::ray_trace_iterator<real_t, int_t> ray_trace_it(
      px, py, dx, dy, min_pt_(0), min_pt_(1),
      cell_size_(0), cell_size_(1));

  real_t dirmag = sqrt(dx*dx + dy*dy); 
  real_t n = floor(max_range * fabs(dx) / dirmag / cell_size_(0)) 
    + floor(max_range * fabs(dy) / dirmag / cell_size_(1));
  int maxsizex = og_.size[0];
  int maxsizey = og_.size[1];

  for (;n > 0; --n, ++ray_trace_it) {

      int i = ray_trace_it->first;
      int j = ray_trace_it->second;

#ifdef DEBUG
      printf("(%d, %d), (%f, %f)\n", i, j, tx, ty);
#endif


      if (i < 0 ||  j < 0 || i >= maxsizex || j >= maxsizey ||
          //(og_.at<uint8_t>(i, j) != FREE)) 
          is_occupied(i, j))
        {
          std::pair<real_t, real_t> final_pos_pair = ray_trace_it.real_position();
          final_pos(0) = final_pos_pair.first;
          final_pos(1) = final_pos_pair.second;

          real_t disp_x = final_pos(0) - px;
          real_t disp_y = final_pos(1) - py;
          reflectance = true;
          return sqrt(disp_x * disp_x + disp_y * disp_y);
      }
  }
  reflectance = false;
  return max_range;
}

#endif // OCCGRID_HPP
