#include <vector>

#include <opencv2/opencv.hpp>
#include <libplayerinterface/player.h>
#include <libplayerc++/playerc++.h>

#include <cvmat_serialization.h>

namespace pc = PlayerCc;

void capture_player_data() {
    pc::PlayerClient robot("localhost");
    pc::Position2dProxy p2d(&robot, 0);
    p2d.RequestGeom();
    player_bbox3d bbox = p2d.GetSize();
    printf("Robot size: (%.3f, %.3f)\n", bbox.sw, bbox.sl);

    pc::LaserProxy laser(&robot, 0);
    laser.RequestGeom();
    player_pose3d_t pose = laser.GetPose();
    printf("Laser pose: (%.3f, %.3f, %.3f)\n", pose.px, pose.py, pose.pz);

    cv::Mat laser_pose_all(0, 3, CV_64F);
    uint32_t count = 0;
    cv::Mat laser_range_all;
    cv::Mat scan_angles_all;
    while (true) {
        if (! robot.Peek(2000)) {
          std::cout << "timeout" << std::endl;
          break;
        }
        robot.Read();
        if ( ! laser.GetCount()) {
            std::cout << "no count" << std::endl;
            continue;
        } else if ( ! count) {
            count = laser.GetCount();
            laser_range_all.create(0, count, CV_64F);
            scan_angles_all.create(0, count, CV_64F);
        }


        std::cout << "(" << p2d.GetXPos() << ", " << p2d.GetYPos() << ")" << std::endl;
        cv::Mat pose = (cv::Mat_<double>(1, 3) << p2d.GetXPos(), p2d.GetYPos(), p2d.GetYaw());
        laser_pose_all.push_back(pose);


        cv::Mat range(1, count, CV_64F);
        cv::Mat angles(1, count, CV_64F);

        for (uint32_t i = 0 ; i < count; i ++) {
            range.at<double>(0, i) = laser.GetRange(i);
            angles.at<double>(0, i) = laser.GetBearing(i);
        }
        laser_range_all.push_back(range);
        scan_angles_all.push_back(angles);
    }
    saveMat(laser_pose_all, "laser_pose_all.bin");
    saveMat(laser_range_all, "laser_range_all.bin");
    saveMat(scan_angles_all, "scan_angles_all.bin");

    cv::Mat laser_pose_all2;
    cv::Mat laser_range_all2;
    cv::Mat scan_angles_all2;
    loadMat(laser_range_all2, "laser_range_all.bin");
    loadMat(laser_pose_all2, "laser_pose_all.bin");
    loadMat(scan_angles_all2, "scan_angles_all.bin");
    cv::Mat diff = (laser_pose_all != laser_pose_all2);
    assert(cv::countNonZero(diff) == 0);
}

int main(int argc, char** argv) {
    capture_player_data();
}
