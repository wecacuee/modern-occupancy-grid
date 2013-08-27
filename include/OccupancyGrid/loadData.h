#include <string>
#include <vector>
#include <gtsam/geometry/Pose2.h>
#include <opencv2/opencv.hpp>

void loadDataFromTxt(const std::string& odometry_fname,
    const std::string& snapshot_fname,
    std::vector<gtsam::Pose2>& allposes,
    std::vector<double>& ranges,
    double& max_dist);
void cvMatToData(const cv::Mat& laser_pose,
    const cv::Mat& laser_range,
    const cv::Mat& scan_angles,
    std::vector<gtsam::Pose2>& allposes,
    std::vector<double>& allranges,
    double& max_dist) ;
void loadPlayerSim(const std::string& laser_pose_file,
    const std::string& laser_range_file,
    const std::string& scan_angles_file,
    std::vector<gtsam::Pose2>& allposes,
    std::vector<double>& allranges,
    double& max_dist) ;
