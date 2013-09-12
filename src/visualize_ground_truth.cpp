#include "OccupancyGrid/cvmat_serialization.h"
#include "OccupancyGrid/occgrid.hpp"
#include <cmath>
#include <opencv2/opencv.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/format.hpp>
#include "OccupancyGrid/loadData.h"
#include <boost/program_options.hpp>

namespace bfs = boost::filesystem;
namespace po = boost::program_options;
using namespace std;
using namespace gtsam;

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

void cvArrow(cv::Mat& vis, 
    int x, int y, int u, int v, cv::Scalar line_color, int line_thickness = 1) 
{
  cv::Point2i p,q;
  q.x = x;
  q.y = y;
  p.x = x + u;
  p.y = y + v;

  cv::line(vis, p, q, line_color, line_thickness, CV_AA, 0);

  const double pi = boost::math::constants::pi<double>();
  double angle = atan2((double) p.y-q.y,(double) p.x-q.x);
  double arrow_length = sqrt(u*u + v*v);
  q.x = (int) (p.x - arrow_length / 3 * cos(angle + (pi / 6)));
  q.y = (int) (p.y - arrow_length / 3 * sin(angle + (pi / 6)));
  cv::line(vis, p, q, line_color, line_thickness, CV_AA, 0);

  q.x = (int) (p.x - arrow_length / 3 * cos(angle - (pi / 6)));
  q.y = (int) (p.y - arrow_length / 3 * sin(angle - (pi / 6)));
  cv::line(vis, p, q, line_color, line_thickness, CV_AA, 0);
}

int main(int argc, char** argv) {

  // parse arguments ///////////////////////////////////////////
  // Declare the supported options.
  po::options_description desc("Visualize data");
  desc.add_options()
    ("help", "produce help message")
    ("width", po::value<double>()->required(), "Width ")
    ("height", po::value<double>()->required(), "Height ")
    ("resolution", po::value<double>()->required(), "Resolution ")
    ("dir", po::value<std::string>()->default_value("."), "Data directory")
;

  po::positional_options_description pos;
  pos.add("width", 1);
  pos.add("height", 1);
  pos.add("resolution", 1);
  pos.add("dir", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  po::notify(vm);    

  double width = vm["width"].as<double>();
  double height = vm["height"].as<double>();
  double resolution = vm["resolution"].as<double>();
  std::string datadirectory = vm["dir"].as<std::string>();
  // end of parse arguments ////////////////////////////////////
  cv::Mat laser_pose, laser_ranges, scan_angles, laser_reflectance;
  loadMat(laser_pose, datadirectory + "/laser_pose_all.bin");
  loadMat(laser_ranges, datadirectory + "/laser_range_all.bin");
  loadMat(scan_angles, datadirectory + "/scan_angles_all.bin");
  loadMat(laser_reflectance, datadirectory + "/laser_reflectance_all.bin");

  std::string floorplanfile = datadirectory + "/floorplan.png";
  cv::Mat floorplan = cv::imread(floorplanfile, cv::IMREAD_GRAYSCALE);

  cv::Vec2d original_size(width, height);
  cv::Vec2i gridsize;
  cv::Vec2d cellsize(resolution, resolution); 
  //cv::divide(original_size, gridsize, cellsize); // cellsize = in meters per pixel
  cv::divide(original_size, cellsize, gridsize);
  cv::resize(floorplan, floorplan, cv::Size(gridsize(0), gridsize(1)),
      0, 0, cv::INTER_AREA);

  double origin_x = 0, origin_y = 0;
  double size_x = width, size_y = height;

    shiftPoses(laser_pose, scan_angles, laser_ranges, laser_reflectance,
        size_x, size_y, origin_x, origin_y);
    std::cout << "Laser spans (" << size_x << ", " << size_y << ")\n";
    std::cout << "Laser count: " << laser_ranges.rows * laser_ranges.cols << "\n";
    cv::Vec2d size_bitmap(size_x, size_y);
    cv::Vec2i size_bitmap_px(size_x / cellsize(0), size_y / cellsize(1));
    cv::Vec2i origin_px(origin_x / cellsize(0), origin_y / cellsize(1));
    cv::Vec2d min_pt(- size_bitmap(0)/2, - size_bitmap(1)/2);
    cv::Vec2i ncells;
    cv::divide(min_pt, cellsize, ncells, -2); // ncells = number of cells in map including margins

    OccupancyGrid2DGT<double, int> map(
        min_pt(0), 
        min_pt(1),
        cellsize(0),
        cellsize(1),
        ncells(0),
        ncells(1));
    map.og_ = cv::Scalar(map.OCCUPIED);

    cv::Vec2i min_floorplan, max_floorplan;
    cv::addWeighted(origin_px, 1, size_bitmap_px, -0.5, 0, min_floorplan);
    cv::addWeighted(origin_px, 1, size_bitmap_px, 0.5, 0, max_floorplan);
    //cv::transpose(floorplan, floorplan);
    cv::Mat fp_roi = floorplan(
        cv::Range(std::max(0, floorplan.rows / 2 - max_floorplan(1)),
                  std::min(floorplan.rows / 2 - min_floorplan(1), floorplan.rows)),
        cv::Range(std::max(0, floorplan.cols / 2 + min_floorplan(0)),
                  std::min(floorplan.cols / 2 + max_floorplan(0), floorplan.cols))
        );

    cv::Vec2i fp_roi_size(fp_roi.cols, fp_roi.rows);
    cv::Vec2i min_bitmap;
    cv::addWeighted(ncells, 0.5, fp_roi_size, -0.5, 0, min_bitmap);
    cv::Vec2i max_bitmap;
    cv::add(min_bitmap, fp_roi_size, max_bitmap);
    printf("Map size (%d, %d)\n", ncells(0), ncells(1));
    printf("(%d, %d) <-> (%d, %d)\n", min_bitmap(0), min_bitmap(1), max_bitmap(0), max_bitmap(1));
    cv::Mat map_roi = map.og_(cv::Range(min_bitmap(0), max_bitmap(0)),
      cv::Range(min_bitmap(1), max_bitmap(1)));

    cv::imshow("fp_roi", fp_roi);
    cv::transpose(fp_roi, fp_roi);
    cv::flip(fp_roi, fp_roi, 1);
    fp_roi.copyTo(map_roi);

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

    // trajectory map
    cv::Mat trajectory;
    cv::cvtColor(map.og_, trajectory, cv::COLOR_GRAY2BGR);
    //const double pi = boost::math::constants::pi<double>();
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
        cv::Vec2d position(pose[0], pose[1]);
        // draw input 
        cv::Mat visin;
        cv::cvtColor(map.og_, visin, cv::COLOR_GRAY2BGR);
        map.draw_lasers(visin, position, robot_angle, angles,
            laser_ranges.ptr<double>(r),
            laser_reflectance.ptr<uint8_t>(r),
            scan_angles.cols,
            CV_RGB(0, 255, 0));

        cv::transpose(visin, visin);
        cv::flip(visin, visin, 0);
        cv::imshow("c", visin);
        cv::imwrite((in_fmter % r).str(), visin);

        // draw trajectory
        double arrow_len = .5;
        if ( r % (int)(arrow_len / cellsize(0)) == 0) {
          double u = arrow_len * cos(robot_angle);
          double v = arrow_len * sin(robot_angle);
          cv::Point2i pt = map.xy2rc(cv::Vec2d(pose[0], pose[1])); 
          cv::Point2i pt2 = map.xy2rc(cv::Vec2d(pose[0] + u, pose[1] + v));
          cvArrow(trajectory, 
              pt.x, pt.y, 
              pt2.x - pt.x, pt2.y - pt.y,
              cv::Scalar(255, 0, 0));
          cv::Mat trajvis;
          cv::transpose(trajectory, trajvis);
          cv::flip(trajvis, trajvis, 0);
          cv::imshow("e", trajvis);
        }

        // draw ground truth
        cv::Mat vis;
        cv::cvtColor(map.gt_, vis, cv::COLOR_GRAY2BGR);
        map.draw_lasers(vis, position, robot_angle, angles, &ranges[0],
            laser_reflectance.ptr<uint8_t>(r),
            scan_angles.cols, CV_RGB(0, 255, 0));
        cv::transpose(vis, vis);
        cv::flip(vis, vis, 0);
        cv::imshow("d", vis);
        cv::imwrite((gt_fmter % r).str(), vis);
        cv::waitKey(1);
    }
    cv::transpose(trajectory, trajectory);
    cv::flip(trajectory, trajectory, 0);
    cv::imwrite("trajectory.png", trajectory);

    cv::Mat visgt;
    cv::cvtColor(map.gt_, visgt, cv::COLOR_GRAY2BGR);
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
        cv::Vec2d position(pose[0], pose[1]);
        // draw trajectory
        double arrow_len = .5;
        if ( r % (int)(arrow_len * 10/ cellsize(0)) == 0) {
          double u = arrow_len * cos(robot_angle);
          double v = arrow_len * sin(robot_angle);
          cv::Point2i pt = map.xy2rc(cv::Vec2d(pose[0], pose[1])); 
          cv::Point2i pt2 = map.xy2rc(cv::Vec2d(pose[0] + u, pose[1] + v));
          cvArrow(visgt, 
              pt.x, pt.y, 
              pt2.x - pt.x, pt2.y - pt.y,
              cv::Scalar(255, 0, 0));
          cv::imshow("d", visgt);
          cv::waitKey(33);
        }
    }
    std::stringstream ss;
    ss << "gt-final.png";
    cv::transpose(visgt, visgt);
    cv::flip(visgt, visgt, 0);
    cv::imwrite(ss.str(), visgt);
}
