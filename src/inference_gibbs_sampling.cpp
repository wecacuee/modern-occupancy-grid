#include <opencv2/opencv.hpp>

#include <sstream>
#include <cstdlib>
#include "OccupancyGrid/cvmat_serialization.h"
#include "OccupancyGrid/forward_sensor_model.h"

// constant prior for now
//double prior_occpupied_to_free(
double prior_log_odds_occpupied_to_free(
        OccupancyGrid2D<double, int>& map,
        int k)
{
    return 0;
}

double probabability_occupied_to_free(
    const std::vector<Observation2D>& observations,
    OccupancyGrid2D<double, int>& map,
    int k) {
    // It is strange how prior must be defined
    // It must be P(m_k = 1| m_{\neg k}) / P(m_k = 0 | m_{\neg k)
    // for symmetry
    // 1st term in Eq (3) merali13icra
    double prior_lodds = prior_log_odds_occpupied_to_free(map, k);


    // Don't want to change the original map, so we will reset this value in
    // the end
    //
    uint8_t original_map_k_value = map.get(k);
      //{
        // map_k value can be changed only in this scope
        // mental reminder :P


        // make the cell occupied
        map.set(k, map.OCCUPIED);

        // 2nd term of Eq (3) merali13icra
        double lodds_occupied = 
          log_odds_observation_given_map_and_all_poses(observations, map);

        // what if the cell is unoccpuied
        // 3rd term of Eq (3) merali13icra
        map.set(k,  map.FREE);
        double lodds_free = 
          log_odds_observation_given_map_and_all_poses(observations, map);

      //}
    // reset the original value
    map.set(k, original_map_k_value);

    double total_lodds = prior_lodds + lodds_occupied - lodds_free;
    // conversion from log odds to probability
    // just after Eq (3) merali13icra
    double normalized_prob_occupied = exp(total_lodds) / (1 + exp(total_lodds));

    return normalized_prob_occupied;
}

uint8_t sample_m_k(
    const std::vector<Observation2D>& observations,
        OccupancyGrid2D<double, int>& map,
        int k) {
    double prob_odds = probabability_occupied_to_free(observations, map, k);

    // choose a number in [0, 1]
    double rn = static_cast<double>(std::rand()) / (RAND_MAX);

    // if probabability is more than choose occupied otherwise free
    return (rn < prob_odds) ? map.OCCUPIED : map.FREE;
}

OccupancyGrid2D<double, int>
inference_gibbs_sampling(const std::vector<Observation2D>& observations, double maxSamples = 20)
{
    int nrows(100), ncols(100);
    OccupancyGrid2D<double, int> map(-9, -9, 18.0/nrows, 18.0/ncols, nrows, ncols);
    int total = map.og_.size[0] * map.og_.size[1];
    for (int i = 0; i < maxSamples; i++) {

        for (int k = 0; k < total; k ++) {
            uint8_t occ_or_free = sample_m_k(observations, map, k);
            map.set(k, occ_or_free);
            printf("i=%d, k=%d, of=%d\n", i, k, occ_or_free);
            // cv::imshow("c", map.og_);
            // cv::waitKey(1);
        }
        std::stringstream filename;
        filename << i << ".png";
        cv::imwrite(filename.str(), map.og_);
    }
    return map;
}

void 
downsample(cv::Mat_<double>& matd,
    cv::Mat& mat,
    int steprows=1, int stepcols=1,
    int ignorerows=0)
{
    int newrows = (mat.rows - ignorerows) / steprows;
    int newcols = mat.cols / stepcols;
    matd.create(newrows, newcols);
    printf("%d, %d <=> %d\n", CV_32F, CV_64F, mat.type());
    assert(mat.type() == CV_64F);
    int row = 0;
    for (int i = 0; i < (mat.rows - ignorerows); i += steprows) {
        double* row_ptr = mat.ptr<double>(i);
        int col = 0;
        for (int j = 0; j < mat.cols; j += stepcols) {
            matd(row, col++) = row_ptr[j];
        }
        row++;
    }
}

/// This method is deprecared. See visualize_ground_truth.cpp
void visualize_input(cv::Mat_<double> laser_pose,
    cv::Mat_<double> laser_range,
    cv::Mat_<double> scan_angles,
    int grid_size)
{
    int nrows(grid_size), ncols(grid_size);
    OccupancyGrid2D<double, int> map(-9, -9, 18.0/nrows, 18.0/ncols, nrows, ncols);
    int nobservations = laser_range.rows;
    int scan_count = laser_range.cols;

    for (int i = 0; i < nobservations; i++) {
        double* l_pose = laser_pose.ptr<double>(i);
        double* l_range = laser_range.ptr<double>(i);
        double* l_angles = scan_angles.ptr<double>(i);
        cv::Vec2d position(l_pose[0], l_pose[1]);
        double robot_angle = l_pose[2];
        cv::Mat vis = cv::imread("cave.png");
        // cv::cvtColor(map.og_, vis, CV_GRAY2BGR);
        int margin = (5 * grid_size / 100);
        cv::Mat vis1(map.og_.rows, map.og_.cols, CV_8UC3);
        cv::Mat vis1_5 = vis1(cv::Range(margin, map.og_.rows - margin),
            cv::Range(margin, map.og_.cols - margin));
        cv::resize(vis, vis1_5, cv::Size(map.og_.rows - 2*margin, map.og_.cols - 2*margin));
        //cv::cvtColor(vis, vis, CV_GRAY2BGR);
        cv::Matx<double, 2, 3> M(0, -1, vis1_5.rows + margin,
                                  1, 0, margin );
        cv::Mat vis2 = vis.clone();
        cv::warpAffine(vis1_5, vis2, M, cv::Size(vis.rows, vis.cols));
        // std::cout << "Current position:(" << position(0) << "," <<
          // position(1) << ")" << std::endl;
        map.draw_lasers(vis2, position, robot_angle, l_angles, l_range,
            scan_count, CV_RGB(0, 255, 0));
        cv::imshow("c", vis2);
        cv::waitKey(100);
        std::stringstream ss;
        ss << "inputstream/" << i << ".png";
        cv::imwrite(ss.str(), vis2);
    }
}

void assert_mat_not_nan(cv::Mat& scan_angles) {
    for (int i = 0; i < scan_angles.size[0]; i++) {
        for (int j = 0; j < scan_angles.size[1]; j++ ) {
            if (std::isnan(scan_angles.at<double>(i, j))) {
                printf("(%d, %d)\n", i, j);
            }
            assert( ! std::isnan(scan_angles.at<double>(i, j)));
        }
    }
}

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cout << "Sample Usage:" << std::endl;
        std::cout << "bin/inference_gibbs_sampling Data/player_sim/laser_pose_all.bin Data/player_sim/laser_range_all.bin Data/player_sim/scan_angles_all.bin" << std::endl;
        exit(1);
    }
    cv::namedWindow("c", cv::WINDOW_NORMAL);
    cv::Mat laser_pose, laser_range, scan_angles;
    loadMat(laser_pose, argv[1]);
    loadMat(laser_range, argv[2]);
    loadMat(scan_angles, argv[3]);

    cv::Mat_<double> laser_pose_d(laser_pose);
    cv::Mat_<double> laser_range_d(laser_range);
    cv::Mat_<double> scan_angles_d(scan_angles);

    // // downsample poses, downsample lasers, skip last few observations
    // downsample(laser_pose_d, laser_pose,   2, 1,  0);
    // downsample(laser_range_d, laser_range, 2, 10, 0);
    // downsample(scan_angles_d, scan_angles, 2, 10, 0);
    
    assert_mat_not_nan(laser_pose_d);
    assert_mat_not_nan(laser_range_d);
    assert_mat_not_nan(scan_angles_d);

    int nposes = scan_angles_d.rows;
    int nlasers = scan_angles_d.cols;
    std::vector<Observation2D> observations(nposes * nlasers);
    for (int r = 0; r < nposes; r++) {
        double* ranges = laser_range_d.ptr<double>(r);
        double* angles = scan_angles_d.ptr<double>(r);
        double* robot_pose = laser_pose_d.ptr<double>(r);
        for (int c = 0; c < nlasers; c++) {
            double laser_angle = robot_pose[2] + angles[c];

            int obs_idx = r * nlasers + c;
            observations[obs_idx] = Observation2D(robot_pose[0],
                robot_pose[1],
                laser_angle,
                ranges[c]);
        }
    }

    inference_gibbs_sampling(observations);
}

