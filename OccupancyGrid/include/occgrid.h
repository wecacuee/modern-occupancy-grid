#ifndef OCCGRID_H
#define OCCGRID_H
// c
#include <cmath>
#include <cassert>
#include <cstdlib>

// std
#include <string>
#include <sstream>
#include <algorithm>
#include <iostream>
#include <stdexcept>

// 3rd party
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

template <typename T>
inline int signum(T val) {
    return (T(0) < val) - (val < T(0));
}

template <typename real_t, typename int_t>
class OccupancyGrid2D {
  public:
    cv::Vec<real_t, 2> min_pt_;
    cv::Vec<real_t, 2> cell_size_;
    cv::Mat og_;
    int counter;
    static const uint8_t OCCUPIED;
    static const uint8_t FREE;

    OccupancyGrid2D(real_t min_x, real_t min_y,
	real_t cell_size_x, real_t cell_size_y,
	int_t ncells_x, int_t ncells_y) :
      min_pt_(min_x, min_y),
      cell_size_(cell_size_x, cell_size_y),
      og_(ncells_x, ncells_y, CV_8U, cv::Scalar(FREE)),
      counter(0)
    {
    }

    /*
     *          ^
     *          |                    
     *      ^   |         ^
     *      |   |       ^ |
     *     cols |      y| |
     *          |         |
     *          |         +------------>
     *          |               x ->
     *          |
     *	        |     
     * min_pt_  +----+----+----+---->
     *                rows ->
     */
    inline bool out_of_bounds(int i, int j) {
        return (i >= og_.size[0] || j >= og_.size[1] || i < 0 || j < 0); 
    }

    virtual bool is_occupied(int_t i, int_t j) {
        return (og_.at<uint8_t>(i, j) != FREE);
    }

    template <typename T> inline T max(T t1, T t2) {
        return (t1 > t2) ? t1 : t2;
    }

    template <typename T> inline T min(T t1, T t2) {
          return (t1 < t2) ? t1 : t2;
    }

    inline void set(int_t k, uint8_t value) {
        int row = k / og_.size[1];
        int col = k % og_.size[1];
        //printf("%d, %d, %d, %lu", k, row, col, og_.total());
        assert(k < og_.total());
        og_.at<uint8_t>(row, col) = value;
    }

    inline uint8_t get(int_t k) const {
        int row = k / og_.size[1];
        int col = k % og_.size[1];
        //printf("%d, %d, %d, %lu", k, row, col, og_.total());
        assert(k < og_.total());
        return og_.at<uint8_t>(row, col);
    }

    real_t nearest_neighbor_distance(
        cv::Vec<real_t, 2> position,
        real_t max_range,
        cv::Vec<int_t, 2>& nearest_neighbor);

    real_t ray_trace(
        real_t px, 
        real_t py,
        real_t ptheta,
        real_t max_range,
        cv::Vec<real_t, 2>& final_pos); 

    cv::Point2i
      xy2rc(cv::Vec<real_t, 2> xy) {
          int row = static_cast<int>((xy(0) - min_pt_(0)) / cell_size_(0));
          int col = static_cast<int>((xy(1) - min_pt_(1)) / cell_size_(1));
          return cv::Point2i(col, row);
      }

    void draw_lasers(
        cv::Mat& vis,
        cv::Vec<real_t, 2> position,
        real_t robot_angle,
        real_t* scan_angles,
        real_t* ranges,
        int_t scan_count,
        const cv::Scalar &color) {

        for (int i = 0; i < scan_count; i ++) {
            real_t laser_angle = scan_angles[i] + robot_angle;
            cv::Vec<real_t, 2> obs_pt2;
            obs_pt2(0) = cos(laser_angle)* ranges[i] + position(0);
            obs_pt2(1) = sin(laser_angle)* ranges[i] + position(1);
            cv::circle(vis, xy2rc(position), 2, CV_RGB(0, 0, 0), -1);
            cv::line(vis, xy2rc(position), xy2rc(obs_pt2),
                color);
                
        }
    }

    void draw_and_show_lasers(cv::Vec<real_t, 2> position,
        real_t robot_angle,
        real_t* scan_angles,
        real_t* expected_ranges,
        real_t* observed_ranges,
        int_t scan_count) {
        cv::Mat vis; 
        cv::cvtColor(og_, vis, CV_GRAY2BGR);
        draw_lasers(vis,
            position,
            robot_angle,
            scan_angles,
            observed_ranges,
            scan_count,
            CV_RGB(0, 255, 0));
        draw_lasers(vis, position, robot_angle, scan_angles, expected_ranges,
            scan_count, CV_RGB(255, 0, 0));
        cv::imshow("c", vis);
        cv::waitKey(10);
    }
};

#endif
