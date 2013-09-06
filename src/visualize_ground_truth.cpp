#include "OccupancyGrid/cvmat_serialization.h"
#include "OccupancyGrid/occgrid.hpp"
#include <cmath>
#include <opencv2/opencv.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/format.hpp>

namespace bfs = boost::filesystem;

/// Override class to override is_occupied function so that it can copy the
/// ground truth map everytime a laser crosses a cell.
template<typename real_t, typename int_t>
class OccupancyGrid2DGT : public OccupancyGrid2D<real_t, int_t> {
  public:
    cv::Mat_<uint8_t> gt_;
    using OccupancyGrid2D<real_t, int_t>::og_;
    using OccupancyGrid2D<real_t, int_t>::FREE;

    OccupancyGrid2DGT(real_t min_x, real_t min_y, real_t cell_size_x, real_t
        cell_size_y, int_t ncells_x, int_t ncells_y) :
      OccupancyGrid2D<real_t, int_t>(min_x, min_y, cell_size_x, cell_size_y,
          ncells_x, ncells_y),
      gt_(ncells_x, ncells_y)
    {
    };

    virtual bool is_occupied(int_t i, int_t j) {
        uint8_t* row = og_.ptr(i);
        uint8_t val = row[j];
        gt_(i, j) = val;
        return (val != FREE);
    }
};

int main(int argc, char** argv) {
  if (argc != 8) {
    std::cout << "Sample usage (from Data/player_sim_with_reflectance):"
      << "../../bin/visualize_ground_truth laser_pose_all.bin laser_range_all.bin scan_angles_all.bin ../player_worlds/bitmaps/cave-rotated.png 16 16 laser_reflectance_all.bin\n";
    std::cout << "Got " << argc << " arguments\n";
    return 1;
  }
    cv::Mat laser_pose;
    loadMat(laser_pose, argv[1]);
    cv::Mat laser_ranges;
    loadMat(laser_ranges, argv[2]);
    cv::Mat scan_angles;
    loadMat(scan_angles, argv[3]);
    cv::Mat floorplan = cv::imread(argv[4], cv::IMREAD_GRAYSCALE);
    cv::Vec2d size_bitmap(atof(argv[5]), atof(argv[6]));
    cv::Mat laser_reflectance;
    loadMat(laser_reflectance, argv[7]);

    cv::Vec2d margin(1, 1);
    cv::Vec2d min_pt(-margin(0) - size_bitmap(0)/2, -margin(1) - size_bitmap(1)/2);
    cv::Vec2i gridsize(floorplan.size[0], floorplan.size[1]);
    cv::Vec2d cellsize; 
    cv::divide(size_bitmap , gridsize, cellsize);
    //std::cout << cellsize(0) << cellsize(1) << std::endl;
    cv::Vec2i ncells;
    cv::divide(min_pt, cellsize, ncells, -2);
    //std::cout << ncells(0) << ncells(1) << std::endl;

    OccupancyGrid2DGT<double, int> map(
        min_pt(0), 
        min_pt(1),
        cellsize(0),
        cellsize(1),
        ncells(0),
        ncells(1));
    cv::Vec2i min_bitmap;
    cv::addWeighted(ncells, 0.5, gridsize, -0.5, 0, min_bitmap);
    cv::Vec2i max_bitmap;
    cv::addWeighted(ncells , 0.5, gridsize, 0.5, 0, max_bitmap);
    printf("Map size (%d, %d)\n", ncells(0), ncells(1));
    printf("(%d, %d) <-> (%d, %d)\n", min_bitmap(0), min_bitmap(1), max_bitmap(0), max_bitmap(1));
    cv::Mat roi = map.og_(cv::Range(min_bitmap(0), max_bitmap(0)),
      cv::Range(min_bitmap(1), max_bitmap(1)));
    map.og_ = cv::Scalar(map.OCCUPIED);
    floorplan.copyTo(roi);

    for (cv::MatIterator_<uint8_t> it = map.gt_.begin();
        it != map.gt_.end();
        it++) {
        *it = 127;
    }
    
    double MAX_RANGE = 8;

    std::vector<double> ranges(scan_angles.cols, MAX_RANGE);
    int r;
    boost::format in_fmter("inputstream/%d.png");
    bfs::create_directory("inputstream");
    boost::format gt_fmter("groundtruth/%d.png");
    bfs::create_directory("groundtruth");
    for (r = 0; r < laser_pose.rows; r++) {
        double* pose = laser_pose.ptr<double>(r);
        double* angles = scan_angles.ptr<double>(r);
        double robot_angle = pose[2];
        for (int c = 0; c < scan_angles.cols; c++) {
            double total_angle = robot_angle + angles[c];

            cv::Vec2d final_pos;
            bool reflectance;
            ranges[c] = map.ray_trace(pose[0], pose[1], total_angle, MAX_RANGE, final_pos, reflectance);
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
        cv::imwrite((in_fmter % r).str(), visin);

        // draw ground truth
        cv::Mat vis;
        cv::cvtColor(map.gt_, vis, cv::COLOR_GRAY2BGR);
        map.draw_lasers(vis, position, robot_angle, angles, &ranges[0],
            laser_reflectance.ptr<uint8_t>(r),
            scan_angles.cols, CV_RGB(0, 255, 0));
        cv::imshow("d", vis);
        cv::imwrite((gt_fmter % r).str(), vis);
        cv::waitKey(1);
    }
    std::stringstream ss;
    ss << "groundtruth/" << ++r << ".png";
    cv::imwrite(ss.str(), map.gt_);
}
