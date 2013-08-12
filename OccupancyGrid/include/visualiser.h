/**
 * @file: visualiser.h
 * @date: June 7, 2013
 * @author: Vikas Dhiman
 */
#pragma once
#include <opencv2/opencv.hpp>
#include <boost/format.hpp>
#include "LaserFactor.h"
#include "utility.hpp"

static const cv::Vec3f GREEN(0, 255, 0);
static const cv::Vec3f WHITE(255, 255, 255);
static const cv::Vec3f BLACK(0, 0, 0);

/**
 * @brief: Visualize the occupancy of cells being affected by highlighed laser
 * cells
 */
class Visualiser {
private:
  /// Opencv image to be visualized
  cv::Mat vis_;
  /// Turn on visualization on demand
  bool enable_show_;
  mutable int count_;

public:
  inline Visualiser() : enable_show_(false), count_(0) { }
  Visualiser(size_t nrows, size_t ncols) : vis_(nrows, ncols, CV_8UC3), enable_show_(false), count_(0) { }

  /// For lazy initialization
  inline void init(size_t nrows, size_t ncols) { vis_.create(nrows, ncols, CV_8UC3); }

  /// Convert Index to x,y coordinates for opencv mat
  inline void idx2xy(const gtsam::Index idx, size_t& x, size_t& y) {
    if (vis_.cols == 0) return;
    y = idx / vis_.cols;
    x = idx % vis_.cols;
  }

  /// Set Occupancy for visualization
  inline void setOccupancy(const LaserFactor::Occupancy& occ) {
    for (int i = 0; i < vis_.rows; i++)
      for (int j = 0; j < vis_.cols; j++)
        vis_.at<cv::Vec3b>(i, j) = (occ.at(i*vis_.cols + j) != 0) ? BLACK : WHITE;
  }

  template <typename Map>
  inline void setMapMarginals(Map& marg) {
    typedef typename Map::mapped_type T;
    T max_ele(0);
    for (int i = 0; i < vis_.rows; i++) {
      for (int j = 0; j < vis_.cols; j++) {
        max_ele = std::max(max_ele, marg[i*vis_.cols + j]);
      }
    }
    max_ele = (max_ele == 0) ? 1 : max_ele;
    std::cout << "max element:" << max_ele << std::endl;

    for (int i = 0; i < vis_.rows; i++) {
      for (int j = 0; j < vis_.cols; j++) {
        uint8_t val = occgrid::todouble(marg[i*vis_.cols + j] / max_ele) * 255;
        val = 255 - val;
        vis_.at<cv::Vec3b>(i, j) = cv::Vec3b(val, val, val);
      }
    }
  }

  template <typename T>
  inline void setMarginals(const std::vector<T>& marg) {
    T max_ele(0);
    for (int i = 0; i < vis_.rows; i++) {
      for (int j = 0; j < vis_.cols; j++) {
        max_ele = std::max(max_ele, marg[i*vis_.cols + j]);
      }
    }
    for (int i = 0; i < vis_.rows; i++) {
      for (int j = 0; j < vis_.cols; j++) {
        uint8_t val = occgrid::todouble(marg[i*vis_.cols + j] / max_ele) * 255;
        val = 255 - val;
        vis_.at<cv::Vec3b>(i, j) = cv::Vec3b(val, val, val);
      }
    }
  }

  /// Highlight given vector of cells
  inline void highlightCells(const std::vector<gtsam::Index> & cells) {
    for (size_t i = 0; i < cells.size(); i++) {
      gtsam::Index idx = cells[i];
      size_t x,y;
      idx2xy(idx, x, y);
      vis_.at<cv::Vec3b>(y, x) = GREEN;
    }
  }

  /// clear the visualization
  inline void reset() { init(vis_.rows, vis_.cols); }

  /// Enable visualization with show function
  inline void enable_show() { enable_show_ = true; }

  /// update the window
  void show(int t = 33) const {
    if (! enable_show_) return;
    cv::imshow("c", vis_);
    boost::format format("/tmp/occgridvis%d.png");
    cv::imwrite((format % count_++).str(), vis_);
    cv::waitKey(t);
  }
};

extern Visualiser global_vis_;
