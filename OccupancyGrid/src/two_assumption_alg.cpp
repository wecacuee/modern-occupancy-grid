#include <occgrid.hpp>
#include <opencv2/opencv.hpp>
#include <cvmat_serialization.h>
#include <boost/format.hpp>

/// Override class to override is_occupied function so that it can copy the
/// ground truth map everytime a laser crosses a cell.
template<typename real_t, typename int_t>
class OccupancyGrid2DInverseSensor : public OccupancyGrid2D<real_t, int_t> {
  public:
    using OccupancyGrid2D<real_t, int_t>::og_;
    using OccupancyGrid2D<real_t, int_t>::cell_size_;
    using OccupancyGrid2D<real_t, int_t>::min_pt_;
    using OccupancyGrid2D<real_t, int_t>::FREE;
    int_t observed_manh_range_;
    cv::Vec<int_t, 2> robot_position_;
    cv::Mat_<real_t> log_odds_map_;
    const real_t LOG_ODDS_OCCUPIED;
    const real_t LOG_ODDS_FREE;

    OccupancyGrid2DInverseSensor(real_t min_x, real_t min_y, real_t cell_size_x, real_t
        cell_size_y, int_t ncells_x, int_t ncells_y) :
      OccupancyGrid2D<real_t, int_t>(min_x, min_y, cell_size_x, cell_size_y,
          ncells_x, ncells_y),
      observed_manh_range_(),
      robot_position_(),
      log_odds_map_(ncells_x, ncells_y, 0.0L),
      LOG_ODDS_OCCUPIED(1.3863),
      LOG_ODDS_FREE(-1.3863)
    {
    };

    void set_up_ray_trace(
        real_t px,
        real_t py,
        real_t ptheta,
        real_t observed_range) {
        robot_position_(0) = 
            (int)floor((px - min_pt_(0)) / cell_size_(0));
        robot_position_(1) =
            (int)floor((py - min_pt_(1)) / cell_size_(1));
        real_t dx_abs = fabs(cos(ptheta));
        real_t dy_abs = fabs(sin(ptheta));
        real_t dmag = sqrt(dx_abs * dx_abs + dy_abs * dy_abs);
        observed_manh_range_ =
          floor(observed_range * dx_abs / dmag / cell_size_(0)) +
          floor(observed_range * dy_abs / dmag / cell_size_(1));
        //printf("-----------------\n");
    }
    
    inline int_t manh_distance(int_t i, int_t j) {
        return std::abs(i - robot_position_(0)) + std::abs(j - robot_position_(1));
    }

    virtual bool is_occupied(int_t i, int_t j) {
        uint8_t val = og_.ptr(i)[j];
        bool retval = (val != FREE);
        int_t d = manh_distance(i, j);
        // update step
        // printf("%d < %d\n", d, observed_manh_range_);
        log_odds_map_(i, j) += 
          (d < observed_manh_range_) ?  LOG_ODDS_FREE
          : (d == observed_manh_range_) ? LOG_ODDS_OCCUPIED
          : 0; // unknown
        return retval;
    }

    inline void show(int r) {
        cv::Mat vis;
        cv::exp(log_odds_map_, vis);
        vis = 1 / (1 + vis);
        vis *= 255;
        vis.convertTo(vis, CV_8U);
        cv::imshow("c", vis);
        //cv::imwrite((boost::format("out-%d.png") % r).str(), vis);
        cv::waitKey(1);
    }
};

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cout << "Sample Usage:" << std::endl;
        std::cout << "bin/two_assumption_alg Data/player_sim/laser_pose_all.bin Data/player_sim/laser_range_all.bin Data/player_sim/scan_angles_all.bin" << std::endl;
        exit(1);
    }
    cv::Mat laser_pose;
    loadMat(laser_pose, argv[1]);
    cv::Mat laser_ranges;
    loadMat(laser_ranges, argv[2]);
    cv::Mat scan_angles;
    loadMat(scan_angles, argv[3]);

    cv::Vec2d min_pt(-9, -9);
    cv::Vec2d range = -2 * min_pt;
    cv::Vec2i gridsize(500, 500);
    cv::Vec2d cellsize; 
    cv::divide(range, gridsize, cellsize);
    //std::cout << cellsize(0) << cellsize(1) << std::endl;
    cv::Vec2i ncells;
    cv::divide(min_pt, cellsize, ncells, -2);

    //std::cout << ncells(0) << ncells(1) << std::endl;

    OccupancyGrid2DInverseSensor<double, int> map(
        min_pt(0), 
        min_pt(1),
        cellsize(0),
        cellsize(1),
        ncells(0),
        ncells(1));

    double MAX_RANGE = 8;

    int r;
    for (r = 0; r < laser_pose.rows; r++) {
        double* pose = laser_pose.ptr<double>(r);
        double* ranges = laser_ranges.ptr<double>(r);
        double* angles = scan_angles.ptr<double>(r);
        double robot_angle = pose[2];
        for (int c = 0; c < scan_angles.cols; c++) {
            double total_angle = robot_angle + angles[c];
            cv::Vec2d final_pos;
            map.set_up_ray_trace(pose[0], pose[1], total_angle, ranges[c]);
            map.ray_trace(pose[0], pose[1], total_angle, MAX_RANGE, final_pos);
        }
        map.show(r);
    }
}
