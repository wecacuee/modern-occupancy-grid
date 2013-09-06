#include "OccupancyGrid/loadData.h"
#include "OccupancyGrid/cvmat_serialization.h"
#include <boost/typeof/typeof.hpp>
#include <cstdio>

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
    if (fscanf(fptr, "%lf %lf %lf", &x, &y, &theta) != 3)
      throw std::runtime_error("error while fscanf");
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

  if (fscanf(fptr, "%lf %lf %lf %lf %lf %d", &min_angle, &max_angle, &step,
      &max_dist, &min_dist, &n_readings) != 6)
    throw std::runtime_error("error while fscanf");

  double range;
  Index it = 0;
  while (!feof(fptr)) {
    double theta = 3.14159 / 2 + pose[it].theta() + min_angle; //compute angle of first laser
    for (int i = 0; i < n_readings && !feof(fptr); i++) {
      Pose2 p(pose[it].y() / 100, pose[it].x(), theta);
      allposes.push_back(p);
      if (fscanf(fptr, "%lf", &range) != 1) //load range
        throw std::runtime_error("error while fscanf");

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
    const cv::Mat& laser_reflectance,
    vector<Pose2>& allposes,
    vector<double>& allranges,
    vector<uint8_t>& allreflectance)
{
  for (int i = 0; i < laser_range.rows; i ++) {
    const double* pose = laser_pose.ptr<double>(i);
    const double* ranges = laser_range.ptr<double>(i);
    const double* angles = scan_angles.ptr<double>(i);
    const uint8_t* reflectance = laser_reflectance.ptr<uint8_t>(i);
    for (int j = 0; j < laser_range.cols; j++) {
      double theta = pose[2] + angles[j];
      Pose2 p(pose[0], pose[1], theta);
      allposes.push_back(p);
      allranges.push_back(ranges[j]);
      allreflectance.push_back(reflectance[j]);
    }
  }
}

void 
downsample(cv::Mat& matout,
    cv::Mat& mat,
    int steprows=1, int stepcols=1,
    int ignorerows=0)
{
    int newrows = (mat.rows - ignorerows) / steprows;
    int newcols = mat.cols / stepcols;
    matout.create(newrows, newcols, CV_64F);
    //printf("%d, %d <=> %d\n", CV_32F, CV_64F, mat.type());
    assert(mat.type() == CV_64F);
    int row = 0;
    for (int i = 0; i < (mat.rows - ignorerows); i += steprows) {
        double* row_ptr = mat.ptr<double>(i);
        int col = 0;
        for (int j = 0; j < mat.cols; j += stepcols) {
            matout.at<double>(row, col++) = row_ptr[j];
        }
        row++;
    }
}

void loadPlayerSim(const std::string& laser_pose_file,
    const std::string& laser_range_file,
    const std::string& scan_angles_file,
    const std::string& laser_reflectance_file,
    std::vector<gtsam::Pose2>& allposes,
    std::vector<double>& allranges,
    std::vector<uint8_t>& allreflectance)
{
  cv::Mat laser_pose, laser_range, scan_angles, laser_reflectance;
  loadMat(laser_pose, laser_pose_file);
  loadMat(laser_range, laser_range_file);
  loadMat(scan_angles, scan_angles_file);
  loadMat(laser_reflectance, laser_reflectance_file);

  // cv::Mat laser_pose_d, laser_range_d, scan_angles_d;
  // // downsample poses, downsample lasers, skip last few observations
  // downsample(laser_pose_d, laser_pose,   2, 1,  0);
  // downsample(laser_range_d, laser_range, 2, 10, 0);
  // downsample(scan_angles_d, scan_angles, 2, 10, 0);
    
  cvMatToData(
      laser_pose, laser_range, scan_angles, laser_reflectance,
      allposes, allranges, allreflectance);
}

void shiftPoses(
    double max_range,
    std::vector<gtsam::Pose2>& allposes,
    double& width,
    double& height) 
{
  BOOST_AUTO(it, allposes.begin());
  double min_x = std::numeric_limits<double>::infinity(),
         max_x = - std::numeric_limits<double>::infinity(),
         min_y = std::numeric_limits<double>::infinity(),
         max_y = - std::numeric_limits<double>::infinity();
  for (; it != allposes.end(); ++it) {
    min_x = std::min(it->x(), min_x);
    min_y = std::min(it->y(), min_y);
    max_x = std::max(it->x(), max_x);
    max_y = std::max(it->y(), max_y);
  }
  double margin_x = 1 + max_range, margin_y = 1 + max_range;
  max_x += margin_x;
  max_y += margin_y;
  min_x -= margin_x;
  min_y -= margin_y;
  double mid_x = (max_x + min_x) / 2;
  double mid_y = (max_y + min_y) / 2;
  it = allposes.begin();
  for (; it != allposes.end(); ++it) {
    *it = gtsam::Pose2( it->x() - mid_x, it->y() - mid_y, it->theta());
  }
  width = max_x - min_x;
  height = max_y - min_y;
}
