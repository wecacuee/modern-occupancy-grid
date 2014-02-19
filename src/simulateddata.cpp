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
#include <boost/program_options.hpp>
namespace po = boost::program_options;

int main(int argc, const char *argv[])
{

  // parse arguments ///////////////////////////////////////////
  // Declare the supported options.
  po::options_description desc("Visualize data");
  desc.add_options()
    ("help", "produce help message")
    ("width", po::value<double>()->required(), "Width ")
    ("height", po::value<double>()->required(), "Height ")
    ("dir", po::value<std::string>()->default_value("."), "Data directory")
;

  po::positional_options_description pos;
  pos.add("width", 1);
  pos.add("height", 1);
  pos.add("dir", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  po::notify(vm);    

  double width = vm["width"].as<double>();
  double height = vm["height"].as<double>();
  std::string datadirectory = vm["dir"].as<std::string>();
  // end of parse arguments ////////////////////////////////////
  cv::Mat laser_pose, laser_ranges, scan_angles;
  loadMat(laser_pose, datadirectory + "/laser_pose_all.bin");
  loadMat(laser_ranges, datadirectory + "/laser_range_all.bin");
  loadMat(scan_angles, datadirectory + "/scan_angles_all.bin");
  cv::Mat laser_reflectance(laser_ranges.rows, laser_ranges.cols, CV_8U);
  std::string floorplanfile = datadirectory + "/floorplan.png";
  cv::Mat floorplan = cv::imread(floorplanfile, cv::IMREAD_GRAYSCALE);
    if(! floorplan.data )                              // Check for invalid input
    {
      std::cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }
  cv::transpose(floorplan, floorplan);
  cv::flip(floorplan, floorplan, 1);
  cv::Vec2d size_bitmap(width, height);
  cv::Vec2d margin(1, 1);
  cv::Vec2d min_pt(- margin(0) - size_bitmap(0)/2, - margin(1) - size_bitmap(1)/2);
  double max_range = 8;
  cv::Vec2i gridsize(floorplan.size[0], floorplan.size[1]);
  cv::Vec2d cellsize; 
  cv::divide(size_bitmap , gridsize, cellsize);
  //std::cout << cellsize(0) << cellsize(1) << std::endl;
  cv::Vec2i ncells;
  cv::divide(min_pt, cellsize, ncells, -2);

  OccupancyGrid2D<double, int> map(
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
