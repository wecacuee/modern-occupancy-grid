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
#include <unistd.h>

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
  std::string window_name_;

public:
  inline Visualiser() : enable_show_(false), count_(0), window_name_("c") { }
  Visualiser(size_t nrows, size_t ncols, std::string winname="c")
    : vis_(nrows, ncols, CV_8UC3), enable_show_(false), count_(0),
    window_name_(winname) { }

  /// For lazy initialization
  inline void init(size_t nrows, size_t ncols, std::string winname="c") {
    vis_.create(nrows, ncols, CV_8UC3); 
    window_name_ = winname;
  }

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

  /// Highlight given vector of cells
  template <typename Range>
  inline void highlightCells(const Range& cells) {
    BOOST_FOREACH(gtsam::Index idx, cells) {
      size_t x,y;
      idx2xy(idx, x, y);
      vis_.at<cv::Vec3b>(y, x) = GREEN;
    }
  }

  template <typename Map, typename T>
    void _setMarginals(Map& marg) {
      T max_ele(0);
      assert( ! isnan(max_ele));
      for (int i = 0; i < vis_.rows; i++) {
        for (int j = 0; j < vis_.cols; j++) {
          T mij = marg[i*vis_.cols + j];
          assert(! isnan(mij));
          max_ele = std::max(max_ele, mij);
          assert(! isnan(max_ele));
        }
      }
      if (max_ele == T(0))
        max_ele = T(1);
      for (int i = 0; i < vis_.rows; i++) {
        for (int j = 0; j < vis_.cols; j++) {
          T normalized = (marg[i*vis_.cols + j]) / max_ele;
          double valdouble = occgrid::todouble(normalized);
          if (! (valdouble <= 1)) {
            std::cout << "Valdouble:" << valdouble 
              << "; ij:" << marg[i*vis_.cols + j] << "; max_ele:" << max_ele
              << std::endl;
            assert(false);
          }

          uint8_t val = valdouble * 255;
          val = 255 - val;
          vis_.at<cv::Vec3b>(i, j) = cv::Vec3b(val, val, val);
        }
      }
    }

  template <typename Map>
  inline void setMapMarginals(Map& marg) {
    _setMarginals<Map, typename Map::mapped_type>(marg);
  }

  template <typename Map>
    inline 
    typename boost::disable_if<boost::is_void<typename Map::mapped_type>, void>::type
    setMarginals(Map& marg) {
      typedef typename Map::mapped_type T;
      _setMarginals<Map, T>(marg);
    }

  template <typename T>
  inline void setMarginals(const std::vector<T>& marg) {
    _setMarginals<const std::vector<T>, T>(marg);
  }

  /// clear the visualization
  inline void reset() { init(vis_.rows, vis_.cols, window_name_); }

  /// Enable visualization with show function
  inline void enable_show() { enable_show_ = true; }

  /// update the window
  void show(int t = 33) const {
    if (! enable_show_) return;
    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    cv::imshow(window_name_, vis_);
    boost::format format("/tmp/occgridvis-%d-%s-%d.png");
    cv::imwrite((format % getpid() % window_name_ % count_++).str(), vis_);
    cv::waitKey(t);
  }

  void save(std::string fname) const {
    cv::imwrite(fname, vis_);
  }
};

extern Visualiser global_vis_;
extern Visualiser global_vis2_;
extern Visualiser global_vis3_;
