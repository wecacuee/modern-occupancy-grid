#include <vector>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include "OccupancyGrid/cvmat_serialization.h"
using namespace std;
int main(int argc, const char *argv[])
{
  ifstream logfile;
  if (argc > 1)
    logfile.open(argv[1]);
  std::string line;

  cv::Mat laser_pose_all(0, 3, CV_64F);
  cv::Mat laser_range_all;
  cv::Mat scan_angles_all;
  cv::Mat laser_reflectance_all;
  bool first_iteration = true;
  while (std::getline(logfile, line)) {
    if (line.length() > 0 && line[0] == '#')
      continue;
    std::istringstream iss(line);
    double timestamp; 
    std::string host; // rossum
    unsigned int port; // 6665
    string interface; // laser, position, sonar
    cv::Mat current_pose(1, 3, CV_64F);
    if (iss >> timestamp >> host >> port >> interface) {
      string index; // laser:0 0 part of the device address
      double timestamp2;
      iss >> index >> timestamp2;

      if (interface == "laser") {
        double min_angle, max_angle, step_size, count;
        iss >> min_angle >> max_angle >> step_size >> count;
        if (first_iteration == true) {
          first_iteration = false;
            laser_range_all.create(0, count, CV_64F);
            scan_angles_all.create(0, count, CV_64F);
            laser_reflectance_all.create(0, count, CV_8U);
        }
        cv::Mat range(1, count, CV_64F);
        cv::Mat angles(1, count, CV_64F);
        cv::Mat reflectance(1, count, CV_8U);
        for(int i = 0; i < count; ++i) {
          double r, intensity;
          iss >> r >> intensity;
          double angle = min_angle + i * (max_angle - min_angle) / (count - 1);
          range.at<double>(0, i) = r;
          angles.at<double>(0, i) = angle;
          reflectance.at<uint8_t>(0, i) = 1;
        }
        laser_pose_all.push_back(current_pose);
        laser_range_all.push_back(range);
        scan_angles_all.push_back(angles);
        laser_reflectance_all.push_back(reflectance);
      } else if (interface == "position") {
        double x, y, theta;
        iss >> x >> y >> theta;
        current_pose.at<double>(0, 0) = x;
        current_pose.at<double>(0, 1) = y;
        current_pose.at<double>(0, 2) = theta;
        double vx, vy, vtheta;
        iss >> vx >> vy >> vtheta;
      }
    } else {
      std::cerr << "Error parsing line:" << line << std::endl;
    }
  }
  saveMat(laser_pose_all, "laser_pose_all.bin");
  saveMat(laser_range_all, "laser_range_all.bin");
  saveMat(scan_angles_all, "scan_angles_all.bin");
  saveMat(laser_reflectance_all, "laser_reflectance_all.bin");
  return 0;
}
