/**
 * Generate ranges and relfectance from poses
 */
#include "OccupancyGrid/occgrid.hpp"
#include <opencv2/opencv.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include "OccupancyGrid/forward_sensor_model.h"
#include "OccupancyGrid/cvmat_serialization.h"

int main(int argc, const char *argv[])
{
  if (argc != 6) {
    std::cout << "Sample usage from OccupancyGrid/Data/ds3:"
      << "../../bin/simulateddata laser_pose_all.bin scan_angles_all.bin ../player_worlds/bitmaps/cave-rotated.png" 
      << std::endl;
    return 1;
  }
  cv::Mat laser_pose;
  loadMat(laser_pose, argv[1]);
  cv::Mat scan_angles;
  loadMat(scan_angles, argv[2]);
  cv::Mat laser_ranges(scan_angles.rows, scan_angles.cols, CV_64F);
  cv::Mat laser_reflectance(scan_angles.rows, scan_angles.cols, CV_8U);
  cv::Mat floorplan = cv::imread(argv[3], cv::IMREAD_GRAYSCALE);
  cv::Vec2d size_bitmap(atof(argv[4]), atof(argv[5]));
  cv::Vec2d margin(1, 1);
  cv::Vec2d min_pt(- margin(0) - size_bitmap(0)/2, - margin(1) - size_bitmap(1)/2);
  double max_range = 8;
  cv::Vec2i gridsize(floorplan.size[0], floorplan.size[1]);
  cv::Vec2d cellsize; 
  cv::divide(size_bitmap , gridsize, cellsize);
  //std::cout << cellsize(0) << cellsize(1) << std::endl;
  cv::Vec2i ncells;
  cv::divide(min_pt, cellsize, ncells, -2);

  OccupancyGrid<double, int> map(
      min_pt(0), 
      min_pt(1),
      cellsize(0),
      cellsize(1),
      ncells(0),
      ncells(1));
  // initialize map with occupancy with floorplan
  for (int r = 0; r < ncells(0); ++r) {
    for (int c = 0; c < ncells(1); ++c) {
      int fp_r = r - margin(0) / cellsize(0);
      int fp_c = c - margin(1) / cellsize(1);
      if ((0 <= fp_r) && (fp_r < floorplan.rows)
          && (0 <= fp_c) && (fp_c < floorplan.cols)) {
        map.og_.at<uint8_t>(r, c) = (floorplan.at<uint8_t>(fp_r, fp_c) > 127) ? map.FREE : map.OCCUPIED;
      } else {
        map.og_.at<uint8_t>(r, c) = map.OCCUPIED;
      }
    }
  }

  boost::mt19937 gen;
  boost::normal_distribution<> norm_dist(1, NOISE_VARIANCE);
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > norm_rand(gen, norm_dist);

  for (int r = 0; r < scan_angles.rows; r++) {
    double* pose = laser_pose.ptr<double>(r);
    double* angles = scan_angles.ptr<double>(r);
    double robot_angle = pose[2];
    for (int c = 0; c < scan_angles.cols; c++) {
      double total_angle = robot_angle + angles[c];
      cv::Vec2d final_pos;
      bool refl;
      double range = map.ray_trace(pose[0], pose[1], total_angle, max_range, final_pos, refl);
      range *= norm_rand();
      laser_ranges.at<double>(r, c) = range;
      laser_reflectance.at<uint8_t>(r, c) = (uint8_t) refl;
    }

    // draw input 
    cv::Mat visin;
    cv::cvtColor(map.og_, visin, cv::COLOR_GRAY2BGR);
    cv::Vec2d position(pose[0], pose[1]);
    map.draw_lasers(visin, position, robot_angle, angles,
        laser_ranges.ptr<double>(r),
        laser_reflectance.ptr<uint8_t>(r),
        scan_angles.cols,
        CV_RGB(0, 255, 0));
    cv::imshow("c", visin);
    cv::waitKey(33);
  }
  saveMat(laser_ranges, "laser_range_all.bin");
  saveMat(laser_reflectance, "laser_reflectance_all.bin");
}
