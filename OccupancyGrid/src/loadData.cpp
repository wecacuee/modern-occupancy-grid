#include "../OccupancyGrid/include/loadData.h"
#include <cstdio>
#include <cvmat_serialization.h>

using namespace std;
using namespace gtsam;

void loadDataFromTxt(const std::string& odometry_fname,
    const std::string& snapshot_fname,
    vector<Pose2>& allposes,
    vector<double>& ranges,
    double& max_dist)
{
  //======== load odometry ==========//
  vector<Pose2> pose;
  FILE *fptr = fopen(odometry_fname.c_str(), "r");
  while (!feof(fptr)) {
    double x, y, theta;
    fscanf(fptr, "%lf %lf %lf", &x, &y, &theta);
    theta *= 3.174159 / 180.0;
    pose.push_back(*(new Pose2(x, y, theta)));
  }
  fclose(fptr);

  //======= load laser scans ===============//
  fptr = fopen(snapshot_fname.c_str(), "r");
  if (fptr == NULL) {
    printf("Cannot open file...exiting\n");
    exit(1);
  }

  double min_angle, max_angle, step;
  double min_dist;
  int n_readings;

  fscanf(fptr, "%lf %lf %lf %lf %lf %d", &min_angle, &max_angle, &step,
      &max_dist, &min_dist, &n_readings);
  double range;
  Index it = 0;
  while (!feof(fptr)) {
    double theta = 3.14159 / 2 + pose[it].theta() + min_angle; //compute angle of first laser
    for (int i = 0; i < n_readings && !feof(fptr); i++) {
      Pose2 p(pose[it].y() / 100, pose[it].x(), theta);
      allposes.push_back(p);
      fscanf(fptr, "%lf", &range); //load range
      ranges.push_back(range);
      theta += step; //increment angle for next laser
    }
    it++;
  }
  fclose(fptr);
  //======= end load laser scans ============//
}

void cvMatToData(const cv::Mat& laser_pose,
    const cv::Mat& laser_range,
    const cv::Mat& scan_angles,
    vector<Pose2>& allposes,
    vector<double>& allranges,
    double& max_dist) 
{
  max_dist = 8; // constant, not good
  for (int i = 0; i < laser_range.rows; i ++) {
    const double* pose = laser_pose.ptr<double>(i);
    const double* ranges = laser_range.ptr<double>(i);
    const double* angles = scan_angles.ptr<double>(i);
    for (int j = 0; j < laser_range.cols; j++) {
      double theta = pose[2] + angles[j];
      Pose2 p(pose[0], pose[1], theta);
      allposes.push_back(p);
      allranges.push_back(ranges[j]);
    }
  }
}

void loadPlayerSim(const string& laser_pose_file,
    const string& laser_range_file,
    const string& scan_angles_file,
    vector<Pose2>& allposes,
    vector<double>& allranges,
    double& max_dist) 
{
  cv::Mat laser_pose, laser_range, scan_angles;
  loadMat(laser_pose, laser_pose_file);
  loadMat(laser_range, laser_range_file);
  loadMat(scan_angles, scan_angles_file);
  cvMatToData(
      laser_pose, laser_range, scan_angles, 
      allposes, allranges, max_dist);
}

